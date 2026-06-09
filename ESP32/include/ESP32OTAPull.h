/*
ESP32-OTA-Pull - a library for doing "pull" based OTA ("Over The Air") firmware
updates, where the image updates are posted on the web.

MIT License

Copyright (c) 2022-3 Mikal Hart

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#pragma once
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Update.h>
#include <WiFi.h>

class ESP32OTAPull
{
public:
    enum ActionType { DONT_DO_UPDATE, UPDATE_BUT_NO_BOOT, UPDATE_AND_BOOT };

    // Return codes from CheckForOTAUpdate
    enum ErrorCode { UPDATE_AVAILABLE = -3, NO_UPDATE_PROFILE_FOUND = -2, NO_UPDATE_AVAILABLE = -1, UPDATE_OK = 0, HTTP_FAILED = 1, WRITE_ERROR = 2, JSON_PROBLEM = 3, OTA_UPDATE_FAIL = 4, MD5_ERROR = 5 };

    // Details of the last WRITE_ERROR (incomplete transfer).
    const char *GetWriteFailReason() const { return _write_fail_reason; }
    int GetWriteFailOffset() const { return _write_fail_offset; }
    int GetWriteFailTotal() const { return _write_fail_total; }

    // Reason for the last OTA_UPDATE_FAIL (Update.begin/end failure).
    const char *GetUpdateFailReason() const { return _update_fail_reason; }

private:
    void (*Callback)(int offset, int totallength) = NULL;
    ActionType Action = UPDATE_AND_BOOT;
    String Board      = ARDUINO_BOARD;
    String Device     = "";
    String Config     = "";
    String CVersion   = "";
    bool DowngradesAllowed = false;
    bool SerialDebug = false;

    // Diagnostics for the last WRITE_ERROR (incomplete transfer): why it stopped
    // and how far it got. Lets the caller distinguish a dropped connection from a
    // flash short-write or a stalled stream.
    const char *_write_fail_reason = "";
    int _write_fail_offset = 0;
    int _write_fail_total = 0;

    // Update.errorString() captured at the OTA_UPDATE_FAIL paths (begin/end), so
    // callers can report it without reaching into the Update library themselves.
    const char *_update_fail_reason = "";

    static int CompareVersionStrings(const char *lhs, const char *rhs)
    {
        const char *lptr = lhs ? lhs : "";
        const char *rptr = rhs ? rhs : "";
        while (*lptr != '\0' || *rptr != '\0') {
            long lval = 0;
            long rval = 0;
            bool lhas = false;
            bool rhas = false;

            while (*lptr != '\0' && *lptr != '.') {
                if (*lptr >= '0' && *lptr <= '9') {
                    lval = (lval * 10) + (*lptr - '0');
                    lhas = true;
                } else if (lhas) {
                    break;
                }
                lptr++;
            }
            while (*lptr != '\0' && *lptr != '.') {
                lptr++;
            }

            while (*rptr != '\0' && *rptr != '.') {
                if (*rptr >= '0' && *rptr <= '9') {
                    rval = (rval * 10) + (*rptr - '0');
                    rhas = true;
                } else if (rhas) {
                    break;
                }
                rptr++;
            }
            while (*rptr != '\0' && *rptr != '.') {
                rptr++;
            }

            if (lval < rval) return -1;
            if (lval > rval) return 1;

            if (*lptr == '.') lptr++;
            if (*rptr == '.') rptr++;
        }
        return 0;
    }

    int DoOTAUpdate(const char* URL, ActionType Action, const char* md5)
    {
        // A previous attempt may have called Update.begin() and then bailed out
        // (dropped connection, stall timeout, write mismatch) without ending or
        // aborting. Update is then stuck "already running" and every begin() below
        // returns false with NO error set ("No Error") until the next reboot.
        // Clear any leftover state so a retry can start cleanly.
        if (Update.isRunning())
            Update.abort();

        HTTPClient http;
	http.useHTTP10(true);		
        http.begin(URL);
    	http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS); //Forces redirect following, enables OTA updates from more online sources, like GitHub releases.

        // Send HTTP GET request
        int httpResponseCode = http.GET();

        if (httpResponseCode == 200)
        {
            int totalLength = http.getSize();

            // this is required to start firmware update process
            if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
                _update_fail_reason = Update.errorString();
                return OTA_UPDATE_FAIL;
            }

            if (md5 && md5[0] != '\0') {
                if (!Update.setMD5(md5)) {
                    if (SerialDebug) {
                        Serial.println("OTA: invalid MD5 string");
                    }
                    Update.abort();
                    return JSON_PROBLEM;
                }
            }

            // create buffer for read
            uint8_t buff[1280] = { 0 };

            // get tcp stream
            WiFiClient* stream = http.getStreamPtr();

            // read all data from server
            int offset = 0;
            uint32_t last_data_ms = millis();
            // Default reason if the loop exits with http.connected() == false.
            const char *fail_reason = "connection closed";
            while (http.connected() && offset < totalLength)
            {
                size_t sizeAvail = stream->available();
                if (sizeAvail > 0)
                {
                    size_t bytes_to_read = min(sizeAvail, sizeof(buff));
                    size_t bytes_read = stream->readBytes(buff, bytes_to_read);
                    size_t bytes_written = Update.write(buff, bytes_read);
                    if (bytes_read != bytes_written)
                    {
                        if(SerialDebug)
			{
				Serial.printf("Unexpected error in OTA: %d %d %d\n", bytes_to_read, bytes_read, bytes_written);
				Serial.printf("Write returned 0? Common causes are:\n");
				Serial.printf("Using merged .bin file instead of just the app .bin from Arduino\n");
				Serial.printf("Flash encryption configuration issues.\n");
			}
                        fail_reason = "flash short-write";
                        break;
                    }
                    offset += bytes_written;
                    last_data_ms = millis();
                    if (Callback != NULL)
                        Callback(offset, totalLength);
                }
                else
                {
                    // No data ready yet. Yield so IDLE/the task watchdog can run;
                    // the original tight spin on available()/connected() starves
                    // IDLE0 during normal TCP gaps -> TASK_WDT reboot mid-download.
                    // Abort if the stream stalls too long so a dead connection
                    // fails cleanly instead of hanging.
                    if (millis() - last_data_ms > 10000)
                    {
                        fail_reason = "stalled (no data 10s)";
                        break;
                    }
                    delay(2);
                }
            }

            if (offset == totalLength)
            {
                if (Update.end(true)) {
                    delay(1000);

                    // Restart ESP32 to see changes
                    if (Action == UPDATE_BUT_NO_BOOT)
                        return UPDATE_OK;
                    ESP.restart();
                } else {
                    _update_fail_reason = Update.errorString();
                    auto err = Update.getError();
                    if (err == UPDATE_ERROR_MD5) {
                        return MD5_ERROR;
                    }
                    return OTA_UPDATE_FAIL;
                }
            }
            // Incomplete transfer (dropped connection / stall timeout / write
            // mismatch). Record diagnostics and abort so Update isn't left
            // "running" and blocking the next attempt's begin().
            _write_fail_reason = fail_reason;
            _write_fail_offset = offset;
            _write_fail_total = totalLength;
            Update.abort();
            return WRITE_ERROR;
        }

        http.end();
        return httpResponseCode;
    }

public:
    /// @brief Return the version string of the binary, as reported by the JSON
    /// @return The firmware version
    String GetVersion()
    {
        return CVersion;
    }

    /// @brief Override the default "Device" id (MAC Address)
    /// @param device A string identifying the particular device (instance) (typically e.g., a MAC address)
    /// @return The current ESP32OTAPull object for chaining
    ESP32OTAPull &OverrideDevice(const char *device)
    {
        Device = device;
        return *this;
    }

    /// @brief Override the default "Board" value of ARDUINO_BOARD
    /// @param board A string identifying the board (class) being targeted
    /// @return The current ESP32OTAPull object for chaining
    ESP32OTAPull &OverrideBoard(const char *board)
    {
        Board = board;
        return *this;
    }

    /// @brief Specify a configuration string that must match any "Config" in JSON
    /// @param config An arbitrary string showing the current configuration
    /// @return The current ESP32OTAPull object for chaining
    ESP32OTAPull &SetConfig(const char *config)
    {
        Config = config;
        return *this;
    }

    /// @brief Specify whether downgrades (posted version is lower) are allowed
    /// @param allow_downgrades true if downgrades are allowed
    /// @return The current ESP32OTAPull object for chaining
    ESP32OTAPull &AllowDowngrades(bool allow_downgrades)
    {
        DowngradesAllowed = allow_downgrades;
        return *this;
    }

    /// @brief Specify a callback function to monitor update progress
    /// @param callback Pointer to a function that is called repeatedly during update
    /// @return The current ESP32OTAPull object for chaining
    ESP32OTAPull &SetCallback(void (*callback)(int offset, int totallength))
    {
        Callback = callback;
        return *this;
    }

    /// @brief Enable extra debugging output on Serial if required.
    void EnableSerialDebug()
    {
        SerialDebug = true;
    }

    /// @brief The main entry point for OTA Update
    /// @param JSON_URL The URL for the JSON filter file
    /// @param CurrentVersion The version # of the current (i.e. to be replaced) sketch
    /// @param ActionType The action to be performed.  May be any of DONT_DO_UPDATE, UPDATE_BUT_NO_BOOT, UPDATE_AND_BOOT (default)
    /// @return ErrorCode or HTTP failure code (see enum above)
    int CheckForOTAUpdate(const char* JSON_URL, const char *CurrentVersion, ActionType Action = UPDATE_AND_BOOT)
    {
        CurrentVersion = CurrentVersion == NULL ? "" : CurrentVersion;

		HTTPClient http;
		http.useHTTP10(true); // Avoid issues with HTTP Chunked Responses
		
		// Send request
		http.begin(JSON_URL);
	    	http.setFollowRedirects(HTTPC_FORCE_FOLLOW_REDIRECTS); //Forces redirect following, enables OTA updates from more online sources, like GitHub releases.
		
        // Send HTTP GET request
        int httpResponseCode = http.GET();
		
        if (SerialDebug) {
            Serial.print("Got HTTP Response: ");
            Serial.println(httpResponseCode, DEC);
        }

        if (httpResponseCode != 200) {
		   return httpResponseCode > 0 ? httpResponseCode : HTTP_FAILED;
		}

		// Get the raw and the decoded stream
		Stream& rawStream = http.getStream();
		
		// Parse response
		JsonDocument doc;
		
		// Parse JSON object (and intercept)
		//ReadLoggingStream loggingStream(rawStream, Serial);
		//DeserializationError error = deserializeJson(doc, loggingStream);
		
		// Parse JSON object
		DeserializationError error = deserializeJson(doc, rawStream);

		// Disconnect
		http.end();		

		if (error) {
            if (SerialDebug)  {
                Serial.print(F("deserializeJson() failed: "));
                Serial.println(error.f_str());
            }
			return JSON_PROBLEM;
		}

        String _Board    = Board.isEmpty() ? ARDUINO_BOARD : Board;
		String _Device   = Device.isEmpty() ? WiFi.macAddress() : Device;
        String _Config   = Config.isEmpty() ? "" : Config;
        bool foundProfile = false;

        if (SerialDebug) {

            Serial.println("Looking for a configuration that matches:");
            Serial.print("Board: ");    Serial.println(_Board);
            Serial.print("Version: ");  Serial.println(CurrentVersion);
            Serial.print("Device: ");   Serial.println(_Device);
        }

        // Step through the configurations looking for a match
        for (auto config : doc["Configurations"].as<JsonArray>())
        {
            String CBoard   = config["Board"].isNull() ? "" : (const char *)config["Board"];
            String CDevice  = config["Device"].isNull() ? "" : (const char *)config["Device"];
            CVersion        = config["Version"].isNull() ? "" : (const char *)config["Version"];
            String CConfig  = config["Config"].isNull() ? "" : (const char *)config["Config"];
            String CMD5     = config["MD5"].isNull() ? "" : (const char *)config["MD5"];

            if ((CBoard.isEmpty() || CBoard == _Board) &&
                (CDevice.isEmpty() || CDevice == _Device) &&
                (CConfig.isEmpty() || CConfig == _Config))
            {
                int versionCmp = CompareVersionStrings(CVersion.c_str(), CurrentVersion);
                if (CVersion.isEmpty() || versionCmp > 0 ||
                    (DowngradesAllowed && versionCmp != 0)) {
                    return Action == DONT_DO_UPDATE ? UPDATE_AVAILABLE : DoOTAUpdate(config["URL"], Action, CMD5.c_str());
                }
                foundProfile = true;
            }
        }
        return foundProfile ? NO_UPDATE_AVAILABLE : NO_UPDATE_PROFILE_FOUND;
    }
};

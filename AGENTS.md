# AGENTS

## Always-on rules
- Always update the conversation log.
- Add new entries in the conversation log on top, but only for actual conceptual or coding work
- Adhere to the coding style of existing sources.
- Prefer separation of concerns where feasible.
- Prefer a single source of truth.
- Avoid duplicated code.
- Suggest useful additions to AGENTS.md 
- Call out when a change relies on async/out-of-order data arrival and how it is handled.
- FFB tuning parameter references shall be normalized to make tuning across models more intuitive.
- Avoid committing build artifacts (e.g., `SimHubPlugin/bin`, `OTA/*.bin`, `OTA/*.ffbota`) unless explicitly requested.
- Update `SimHubPlugin/Docs/FFB_Design_Current.md` whenever the FFB design changes.
- When extending CAN payloads, document frame sizing and update both pack/unpack and cache handling.
- Maintain a short "Commit highlights" list in the latest conversation log entry and confirm it before committing.

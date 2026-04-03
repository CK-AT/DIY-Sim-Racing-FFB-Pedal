using System;
using System.Collections.Generic;
using SharpDX.DirectInput;

namespace DiyFfb
{
    /// <summary>
    /// Reads joystick buttons and keyboard keys via DirectInput for use as graph inputs.
    /// Supports multiple joystick devices simultaneously.
    /// </summary>
    public sealed class ButtonInputReader : IDisposable
    {
        private readonly DirectInput _directInput;
        private readonly Dictionary<string, Joystick> _joysticks = new Dictionary<string, Joystick>();
        private readonly Dictionary<string, bool[]> _joystickStates = new Dictionary<string, bool[]>();
        private Keyboard _keyboard;
        private KeyboardState _keyboardState;
        private bool _disposed;

        public ButtonInputReader()
        {
            _directInput = new DirectInput();
        }

        /// <summary>
        /// Polls all acquired devices and updates cached button/key states.
        /// Call once per graph evaluation cycle.
        /// </summary>
        public void Poll()
        {
            if (_disposed) return;

            // Ensure devices are acquired (must happen on the same thread that created DirectInput)
            EnsureKeyboard();
            EnsureActiveBindings();

            // Poll keyboard
            if (_keyboard != null)
            {
                try
                {
                    _keyboard.Poll();
                    _keyboardState = _keyboard.GetCurrentState();
                }
                catch
                {
                    // Device lost — will be re-acquired on next EnsureDevice call
                    DisposeKeyboard();
                }
            }

            // Poll joysticks
            var lostDevices = new List<string>();
            foreach (var kv in _joysticks)
            {
                try
                {
                    kv.Value.Poll();
                    var state = kv.Value.GetCurrentState();
                    _joystickStates[kv.Key] = state.Buttons;
                }
                catch
                {
                    lostDevices.Add(kv.Key);
                }
            }
            foreach (var guid in lostDevices)
            {
                _joysticks[guid]?.Dispose();
                _joysticks.Remove(guid);
                _joystickStates.Remove(guid);
            }
        }

        /// <summary>
        /// Returns whether the given binding is currently pressed.
        /// </summary>
        public bool IsPressed(ButtonBinding binding)
        {
            if (binding == null || binding.Type == BindingType.None) return false;

            switch (binding.Type)
            {
                case BindingType.JoystickButton:
                    return GetJoystickButton(binding.DeviceInstanceGuid, binding.ButtonIndex);
                case BindingType.KeyboardKey:
                    return GetKeyboardKey(binding.KeyCode);
                default:
                    return false;
            }
        }

        private bool GetJoystickButton(string deviceGuid, int buttonIndex)
        {
            if (string.IsNullOrEmpty(deviceGuid) || buttonIndex < 0) return false;

            // Only read cached state — don't call EnsureJoystick here.
            // Devices are acquired during Poll() which runs on the correct thread.
            if (_joystickStates.TryGetValue(deviceGuid, out var buttons)
                && buttonIndex < buttons.Length)
            {
                return buttons[buttonIndex];
            }
            return false;
        }

        private bool GetKeyboardKey(int keyCode)
        {
            if (keyCode < 0) return false;

            // Only read cached state — don't call EnsureKeyboard here.
            if (_keyboardState == null) return false;

            try
            {
                return _keyboardState.IsPressed((Key)keyCode);
            }
            catch
            {
                return false;
            }
        }

        private Dictionary<string, ButtonBinding> _activeBindings;

        /// <summary>
        /// Sets the active bindings so Poll() can acquire the necessary devices.
        /// Called from BuildGraphInputs on the polling thread.
        /// </summary>
        public void SetActiveBindings(Dictionary<string, ButtonBinding> bindings)
        {
            _activeBindings = bindings;
        }

        private void EnsureActiveBindings()
        {
            if (_activeBindings == null) return;
            foreach (var binding in _activeBindings.Values)
            {
                if (binding.Type == BindingType.JoystickButton)
                    EnsureJoystick(binding.DeviceInstanceGuid);
            }
        }

        private void EnsureKeyboard()
        {
            if (_keyboard != null || _disposed) return;
            try
            {
                _keyboard = new Keyboard(_directInput);
                _keyboard.Properties.BufferSize = 16;
                _keyboard.Acquire();
            }
            catch
            {
                _keyboard = null;
            }
        }

        private void EnsureJoystick(string deviceGuid)
        {
            if (_joysticks.ContainsKey(deviceGuid) || _disposed) return;
            if (!Guid.TryParse(deviceGuid, out var guid)) return;

            try
            {
                var joystick = new Joystick(_directInput, guid);
                joystick.Properties.BufferSize = 16;
                joystick.Acquire();
                _joysticks[deviceGuid] = joystick;
            }
            catch
            {
                // Device not available — skip silently
            }
        }

        /// <summary>
        /// Enumerates all connected DirectInput joystick/gamepad devices.
        /// Used by the binding UI to discover available devices.
        /// </summary>
        public IList<DeviceInstance> EnumerateJoysticks()
        {
            if (_disposed) return Array.Empty<DeviceInstance>();
            var devices = new List<DeviceInstance>();
            foreach (var device in _directInput.GetDevices(DeviceClass.GameControl, DeviceEnumerationFlags.AttachedOnly))
            {
                devices.Add(device);
            }
            return devices;
        }

        /// <summary>
        /// Scans all joysticks and keyboard for any newly pressed button/key.
        /// Returns the binding if found, or null if nothing was pressed.
        /// Used by the press-to-bind UI flow.
        /// </summary>
        public ButtonBinding ScanForPress()
        {
            if (_disposed) return null;

            // Check keyboard
            EnsureKeyboard();
            if (_keyboardState != null)
            {
                foreach (Key key in Enum.GetValues(typeof(Key)))
                {
                    if (key == Key.Escape) continue;  // reserved for cancel
                    try
                    {
                        if (_keyboardState.IsPressed(key))
                        {
                            return new ButtonBinding
                            {
                                Type = BindingType.KeyboardKey,
                                KeyCode = (int)key
                            };
                        }
                    }
                    catch { }
                }
            }

            // Check all connected joysticks
            foreach (var device in _directInput.GetDevices(DeviceClass.GameControl, DeviceEnumerationFlags.AttachedOnly))
            {
                var guidStr = device.InstanceGuid.ToString();
                EnsureJoystick(guidStr);
                if (_joystickStates.TryGetValue(guidStr, out var buttons))
                {
                    for (int i = 0; i < buttons.Length; i++)
                    {
                        if (buttons[i])
                        {
                            return new ButtonBinding
                            {
                                Type = BindingType.JoystickButton,
                                DeviceInstanceGuid = guidStr,
                                DeviceName = device.ProductName ?? device.InstanceName ?? "Unknown",
                                ButtonIndex = i
                            };
                        }
                    }
                }
            }

            return null;
        }

        private void DisposeKeyboard()
        {
            try { _keyboard?.Unacquire(); } catch { }
            try { _keyboard?.Dispose(); } catch { }
            _keyboard = null;
            _keyboardState = null;
        }

        public void Dispose()
        {
            if (_disposed) return;
            _disposed = true;

            DisposeKeyboard();
            foreach (var joystick in _joysticks.Values)
            {
                try { joystick.Unacquire(); } catch { }
                try { joystick.Dispose(); } catch { }
            }
            _joysticks.Clear();
            _joystickStates.Clear();
            _directInput?.Dispose();
        }
    }
}

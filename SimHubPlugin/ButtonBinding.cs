namespace DiyFfb
{
    public enum BindingType { None, JoystickButton, KeyboardKey }

    /// <summary>
    /// Maps a logical grip signal to a physical joystick button or keyboard key.
    /// Persisted in DiyFfbPluginSettings.GripButtonBindings.
    /// </summary>
    public class ButtonBinding
    {
        public BindingType Type = BindingType.None;

        // Joystick fields (used when Type == JoystickButton)
        public string DeviceInstanceGuid = "";
        public string DeviceName = "";
        public int ButtonIndex = -1;

        // Keyboard field (used when Type == KeyboardKey)
        public int KeyCode = -1;

        public string DisplayName
        {
            get
            {
                switch (Type)
                {
                    case BindingType.JoystickButton:
                        return $"{DeviceName}: Btn {ButtonIndex}";
                    case BindingType.KeyboardKey:
                        return $"Keyboard: {((SharpDX.DirectInput.Key)KeyCode)}";
                    default:
                        return "(not bound)";
                }
            }
        }
    }
}

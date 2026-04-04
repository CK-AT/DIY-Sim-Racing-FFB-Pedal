namespace DiyFfb
{
    public enum BindingType { None, JoystickButton, KeyboardKey, HatDirection }

    public enum HatDir { Up = 0, Right = 9000, Down = 18000, Left = 27000 }

    /// <summary>
    /// Maps a logical grip signal to a physical joystick button, keyboard key,
    /// or hat switch direction.
    /// Persisted in DiyFfbPluginSettings.GripButtonBindings.
    /// </summary>
    public class ButtonBinding
    {
        public BindingType Type = BindingType.None;

        // Joystick fields (used when Type == JoystickButton or HatDirection)
        public string DeviceInstanceGuid = "";
        public string DeviceName = "";
        public int ButtonIndex = -1;

        // Keyboard field (used when Type == KeyboardKey)
        public int KeyCode = -1;

        // Hat switch fields (used when Type == HatDirection)
        public int HatIndex = 0;
        public HatDir HatDirection = HatDir.Up;

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
                    case BindingType.HatDirection:
                        return $"{DeviceName}: Hat{HatIndex} {HatDirection}";
                    default:
                        return "(not bound)";
                }
            }
        }
    }
}

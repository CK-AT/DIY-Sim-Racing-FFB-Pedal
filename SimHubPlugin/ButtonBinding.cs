using System;

namespace DiyFfb
{
    public enum BindingType { None, JoystickButton, KeyboardKey, HatDirection }

    public enum HatDir { Up = 0, Right = 9000, Down = 18000, Left = 27000 }

    /// <summary>
    /// DEPRECATED (plan 11): grip signals now bind through SimHub's standard control
    /// panel as named actions (Grip.TrimHat.Up, etc.). This type is retained only so
    /// existing settings.json files from older plugin versions still deserialize.
    /// Slated for removal one minor release after plan 11 ships.
    /// </summary>
    [Obsolete("Replaced by SimHub control bindings — see plan 11. Kept for one release for settings deserialization.")]
    public class ButtonBinding
    {
        public BindingType Type = BindingType.None;
        public string DeviceInstanceGuid = "";
        public string DeviceName = "";
        public int ButtonIndex = -1;
        public int KeyCode = -1;
        public int HatIndex = 0;
        public HatDir HatDirection = HatDir.Up;
    }
}

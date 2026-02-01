namespace DiyFfb.ProfileBrowser
{
    /// <summary>
    /// Defines the context in which the Profile Browser dialog is opened.
    /// </summary>
    public enum ProfileBrowserMode
    {
        /// <summary>
        /// New vehicle detected - user selects a starting template or copies from existing profile.
        /// </summary>
        NewVehicle,

        /// <summary>
        /// User clicked "Manage Profiles" - browse, delete, export stored profiles.
        /// </summary>
        ManageProfiles,

        /// <summary>
        /// User wants to copy tuning from another vehicle to the current one.
        /// </summary>
        CopyFromVehicle
    }
}

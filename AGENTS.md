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
- add unit tests for new features whenever feasable
- Keep graph runtime logic in a shared evaluator and reuse it for editor previews/tests to avoid divergence.
- Keep `SimHubPlugin/Docs/FFB_Graph_Progress.md` updated with Done/In Progress/Open items whenever graph work changes.
- Always adhere to the project's current C# language version and .NET target framework.
- Ensure the SimHub plugin builds as a self-contained DLL with all dependencies merged via ILRepack (Release builds only). Any new package references must be added to the ILRepack merge list in the post-build event.
- **UI Debugging Protocol**: When asked to fix UI appearance issues (e.g., "make it look like X"), IMMEDIATELY: (1) Read the working example's XAML/code to identify styles, templates, and properties; (2) Read the broken code to compare; (3) Fix the most direct cause (control style/properties) before investigating containers/parents; (4) Ask for visual specifics if unclear what's wrong. NEVER make changes based on assumptions without comparing working vs broken code first.

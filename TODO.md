# OpenUSD importer

- [x] Explore the photorealism plugin and existing environment importer.
- [x] Visual companion not needed for the scope and compatibility decisions.
- [x] Clarify intended use: static environments from standard OpenUSD, with an existing public scene as an acceptance target.
- [x] Compare approaches; offline Python conversion selected.
- [x] Obtain approval of the core importer design.
- [x] Approve ALab as the public sample target with documented compatibility limits.
- [x] Write the approved design specification.
- [x] Self-review the specification for placeholders, contradictions, scope, and ambiguity.
- [x] Obtain user approval of the written specification.
- [x] Create and self-review the implementation plan.
- [x] Obtain user review of the plan and select an execution method: Native.
- [x] Implement stage composition, geometry, materials, lights, scene assembly and portable CLI (Tasks 1–6).
- [x] Implement and unit-test optional render/Jolt smoke runner (Task 7 offline portion).
- [x] Build ARGoS/Filament locally and run real synthetic render/Jolt smoke.
- [x] Obtain explicit opt-in and safely download/compose ALab structure and 9.6 GB Techvar assets; document setup/notices.
- [x] Complete full render-purpose ALab conversion, independently reopen both GLBs, and verify actual headless rendering, floor ray and supported foot-bot pose (Task 8). Interactive viewer remains unvalidated: no display.
- [x] Complete fresh independent whole-branch review; reproduce and fix confirmed findings.
- [x] Review follow-up fixes (no unresolved source findings) and finish full-target headless verification. Offline suite: 72 passing tests; engine suite: 45 entries, zero failures, one display-dependent skip. ALab calibrated PNGs and source-derived floor/robot checks pass.

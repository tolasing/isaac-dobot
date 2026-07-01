# RMPflow config (deferred)

CLAUDE.md lists this directory as an "optional Lula/RMPflow config" — cuRobo
(see `../curobo/`) is the primary IK / motion-generation path for this
project, per the top-level CLAUDE.md.

An RMPflow config would need its own Lula robot description (XRDF)
generation step, which is a materially bigger and less certain effort than
the rest of the scaffolding added alongside this file. Deferred until
there's a concrete need for RMPflow specifically (e.g. reactive
obstacle-avoidance behavior cuRobo's `plan_single_js` doesn't cover).

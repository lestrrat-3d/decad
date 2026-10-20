# decad file layout: internal packages, prism and shell

Internal packages for prism and stacked records, prism Booleans and patterns, and Shell.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/prismboolean/` | Analytic boolean admission over payloads: the prism pair with its holed `Intersect` route, axial-Loft and face-patch `Union`, the many-body prism `Union`, blind, spanning and crossing-blind `Cut`, A5 prism groups and A1 stacked `Union`. See prism-boolean §3, general-boolean §3 A1, A5, A7. |
| `internal/stackedunion/` | Builds A1 stacked Union slabs, private scenes, and BRep faces after `internal/prismboolean` admits them. See general-boolean §3 A1. |
| `internal/stackedshell/` | Admits and records the exact stacked shells (nested rectangular boss, circular boss, rectangular blind pocket, coaxial counterbore) from a two-slab record and its face view, and `RefuseClosed`; `breprecord` builds the result. See modify-general §3.1a-§3.1c, §3.1e. |
| `internal/prismshell/` | Prism cap-shell gates with tube, band and closed-shell records; side sections, the side-opening prism, wall bands, nested boss mesh and proof, closed stack reversal, slab levels and stack faces. See modify §8, shell-opening §3–§5, shell-faceted-boss design. |
| `internal/prismrecord/` | The prism record and its builder over nodes (`Build`, `Placed`, extents, map charge); plane-local lifts, point bounds, lift factor, centroid envelope, placement operand, cap frames, mass record, exact box, section-plane test, exact-section gate. Root `prismPayload` aliases it. See evaluator §5, §8. |
| `internal/prismtess/` | Builds a prism payload's chorded mesh node and its face, area and occupied-volume bounds. See tessellation §5. |
| `internal/stackedrecord/` | Stacked record and node builder (`Build`, `Placed`, `Plan`, extents, runs, bounds); builds blind-cut slabs through `prismcells`; derives and audits interfaces, wall columns, exposed patches and the stacked mesh. Root `stackedPrismPayload` aliases it. See stacked-prism §2.2–§5, evaluator §8. |
| `internal/prismextent/` | Prism and profile boundary extremes, bounds and extent readings. See evaluator §5. |
| `internal/prismplacement/` | Exact prism axis shifts, sweep spans, relative placement and exact rectangular boxes in a reference frame. See prism-boolean §3. |
| `internal/prismcells/` | Analytic prism entry and shared pair admission, scene construction, §4.2 union, cut, intersect and crossing resolution, merge audits, cells, charges, trim walks, the placed-axis rectangle gate, overlap volume. See prism-boolean §4, §6, G3. |
| `internal/prismpattern/` | Prism-pattern Union blank and copy authentication, radial source gaps, wall certificates, fillet contacts, ring cuts, polar sectors, clearance, rounded shell and caps, wall mesh, root chords and section separation. See prism-pattern-union design. |
| `internal/cupwall/` | The cup record and view, its node builder (`Build`, `Placed`, cup roles and face order), the `PlanCapShell` offset stage and the cup wall theorem and morphology recheck. Root `cupPayload` and `cupView` alias it. See modify-reach §9.1, payload verification §4, evaluator §8. |
| `internal/shellsurvey/` | Shell inradius, section cavity limit, work budget and contained-disk witness. See modify §8. |
| `internal/shellseparation/` | Bounded source-shell and held-copy separation, exact slab projections, and placement error. See faceted-vertex-bounds §3.7. |

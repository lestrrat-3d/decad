# decad file layout: internal packages, Loft

Internal packages for Loft records, meshes, point and cone sources, fillets, and the cylinder and contact Booleans.
The rules for rows live in `docs/layout.md`.

## Layout

### Internal packages

| Path | Responsibility |
|---|---|
| `internal/pointcone/` | Point-loft cone crossing samples, far-cap subdivision and contact split, cap and triangle crossing bounds, and sector cap paths, subdivision, merged angles and root-edge order, and the strict-side offset of a cone meridian (`offset.go`). See loft-point and loft-face-sections designs. |
| `internal/pointloft/` | Builds the point-section Loft apex fan and proves its source stations, boundary, area and volume. See loft-point design. |
| `internal/loftrecord/` | The loft and chain loft payloads and their builders over nodes: records, evaluation, placement, flat-triangle topology, certified circular-hole walls and bore proofs, mass readings, mesh restatement. Root `loftPayload` and `chainLoftPayload` alias it. See loft §5, §7, §16, §18, evaluator §8. |
| `internal/loftmesh/` | Loft and chain mesh assembly, point-plane and LoftFaces cap layouts and proofs, stations, bounds, split and collar grids, triangle clearance, and contact scans. See loft-point and tessellation designs. |
| `internal/smoothloft/` | Three-section homothety, constant curved-section admission and independent rectangles with exact height curves for Section; varying curves and four-section rounded rectangles. See loft-sections design. |
| `internal/loftfillet/` | Loft fit/circle root-fillet candidate, tangent-root certificate, connector and carrier departures, section contact audits and circle-bore containment. See loft §17–§18. |
| `internal/loftcylinder/` | Cap and source-circle audits, cap tags, ring occupancy, mirrored blank, pairs and builds, Join seams, bore Cut, cap Chamfer admission and builds, mirrored Chamfer assembly, Union proof teeth, local rebuilds, tooth checks, shell and bounds. See loft-cylinder-boolean and loft-mirror-boolean designs. |
| `internal/loftcontact/` | Varying Loft/prism carrier, hull, exact arithmetic, source partition and paired cap certificates, the output shell with its caps, walls, audit, embedding and volume, quarter-Loft held seams and pair separation, held cap outers, and cap, interval and wall records. See general-boolean. |

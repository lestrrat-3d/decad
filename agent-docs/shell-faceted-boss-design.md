# Faceted Union nested-boss Shell

This route shells the complete top cap of one ordinary two-prism mesh Union.
It covers a rectangular plate and a strictly nested rectangular boss whose
horizontal placement rotates and may carry the stored similarity scale. It
rebuilds a faceted cup from the Union's authenticated sources and held mesh.

## Admission

The Union must retain its two original `prismPayload` sources and remain
unplaced. The plate is an exact positive XY rectangular prism with identity
placement. The boss is another exact positive XY rectangular prism. Its
placement has horizontal basis `(c,s),(-s,c)` and unchanged positive Z;
`sqrt(c²+s²)` is enclosed within `[0.99,1.01]`. No source section or level
displacement is admitted. The selected face must be exactly the boss's
`CapEnd`: one group of two triangles, four source corners, and no other
fragment of that cap. All four true boss corners, widened by their source
point bounds, must lie strictly inside the plate's inset by thickness `t`.

The boss bottom is exactly `t` below the plate top. The plate floor below it
and the boss's two horizontal dimensions must leave positive material after
the offset. All levels and offset coordinates used in the reconstruction
must be represented exactly where the route does not carry a numeric bound.
Other Union shapes and placed Union results keep the ordinary Shell refusal.

## Cavity and held boundary

Write `L=sqrt(c²+s²)`, and let `W,H` be the boss's local rectangle widths.
The true boss width and height are `LW,LH`. A world-distance offset `q`
therefore becomes a local inset `q/L`. The cavity has a plate-inset floor at
`z_plate_bottom+t`, lower walls up to `z_boss_bottom`, a ledge from the plate
inset to the boss footprint, the rounded root band up to the plate top,
upper boss-inset walls, and a rim around the removed top cap.

On the root interval, with `s=z-z_boss_bottom`, the true world inset is

```text
q(s) = t - sqrt(t² - s²),       0 ≤ s ≤ t.
```

The held band uses 16 stations at `w=k/16` with rational coordinates
`s=2tw/(1+w²)` and `q=2tw²/(1+w²)`. The square-root bracket for `L` and the
rational-to-float station error enter each new vertex's source point bound.
The planar curve `(q(w),s(w))` has second-derivative norm at most `4t`.
Linear interpolation at the same `w` across one station cell differs by at
most `t/(2·16²)`. Each rectangle corner moves in two inward coordinates,
so the payload charges `3t/(4·16²)` for the curved band. Parameter matching
allows Z to move near the horizontal tangent. The root volume
bound uses swept surface area rather than a same-Z inset difference. The
old Union mesh loses only the selected top cap triangles. Its other facets
and vertex bounds remain unchanged.

The held mesh is checked for positive facet area, exact facet contact,
directed-edge closure, components, and signed volume before the new body is
built. Strict source containment and the monotone nested rectangle sections
keep the new cavity apart from the exterior. A failed audit leaves the Union
live.

## Occupied volume and readings

The true outer volume is plate area times plate height plus `L²WH` times the
boss height above the plate. The lower and upper cavity prisms have ordinary
rectangle volumes. Integrating the root rectangle area gives

```text
V_root = WHL²t - 2(W+H)Lt² + (W+H)Lπt²/2 + (20/3 - 2π)t³.
```

The code evaluates this expression as a rational interval with an outward
interval for `L` and `π`. Its held signed volume must lie inside the stated
occupied-volume allowance. The allowance adds the source Union's symmetric
difference bound, a root-surface sweep term below four times perimeter times
thickness times the chord and station gap, an upper-wall station term, and a
level-slab term. These
terms bound the actual symmetric difference; the final volume comparison is
an independent consistency gate. The faceted result carries the maximum
source and cavity boundary displacement. Its area slack includes both sides
of a conservative root-patch area bound.

# Planar convexity pre-test design

`planar.PlanarConvex` (`internal/pair/planar/planar_convex.go`) proves the
convexity certificate of `agent-docs/multibody-dynamics-design.md` §9.2: every
vertex lies on or behind every triangle's plane. For each triangle with exact
outward normal `n` and anchor corner `a`, and for each vertex `v`, it needs the
sign of the exact dot `E = n·(v − a)`. The certificate fails at the first
positive `E`. This document owns the float pre-test that skips the exact dot
where floats prove `E ≤ 0`, its error bound, its range guard, and why the
certificate's answer and poll count equal those of the all-exact loop.

## Pre-test

**The pre-test only skips an exact dot it proves at or below zero.** Every
other pair takes the exact dot, as the all-exact loop does.

Per solid, once (`newConvexFilter`):

- `v̂` is each vertex coordinate rounded to nearest (`Dyadic.Float64`).
- `R_i` is at or above `|v̂_i − v_i|` over every vertex on axis `i`: the maximum
  of `proof.DyadicFloatError` over the inexact coordinates, and zero when every
  coordinate on the axis is a float.
- `D_i` is at or above `max v̂_i − min v̂_i` (`subUp`).

Per triangle (`convexFilter.plane`), from the exact normal the loop already
forms:

- `n̂_i` is each normal component rounded to nearest.
- `ν_i` is at or above `|n̂_i − n_i|` (`proof.DyadicFloatError`, zero when
  the component is a float).
- `bound` is the error bound below, every operation rounded upward with the
  package's directed operations (`mulUp`, `addUp`; `TestDirectedOpsBracketExact`
  holds them).

Per vertex (`convexPlane.behind`), in plain float64 arithmetic:

```
d_i = v̂_i − â_i
s   = (fl(n̂_0·d_0) + fl(n̂_1·d_1)) + fl(n̂_2·d_2)
skip the exact dot when s ≤ −bound
```

Each product is converted with `float64(...)` before the sum. The Go spec lets
an implementation fuse `x*y + z` into one rounding unless an explicit
conversion separates them, and the arm64 compiler does fuse. With the
conversions every platform rounds the same operations the same way, so amd64
and arm64 reach the same decisions. The bound uses no tuned literal: its constants are `5ε`
and `2⁻¹⁰⁷³` from the derivation below, both exact floats.

## Error bound

**`|s − E| ≤ B = Σ_i [ |n̂_i|·(5ε·D_i + 2R_i) + ν_i·(D_i + 2R_i) ] + 2⁻¹⁰⁷³`,
with `ε = 2⁻⁵³`.** Inside the range guard every float below is finite, so
`E ≤ s + B ≤ s + bound`, and `s ≤ −bound` proves `E ≤ 0`.

Write `e_i = v_i − a_i` (exact) and `ê_i = v̂_i − â_i` (real, unrounded). The
rounding model is round to nearest with unit roundoff `ε`:

1. `|ê_i − e_i| ≤ 2R_i` and `|ê_i| ≤ D_i`, since both vertices are vertices of
   the solid. Hence `|e_i| ≤ D_i + 2R_i`.
2. `d_i = fl(ê_i)` has `|d_i − ê_i| ≤ ε|ê_i|`. A float difference that falls
   into the subnormals is exact, so no absolute term arises.
   Hence `|d_i − e_i| ≤ εD_i + 2R_i` and `|d_i| ≤ (1+ε)D_i`.
3. `P_i = n̂_i·d_i` (real). Then `P_i − n_i e_i = n̂_i(d_i − e_i) + (n̂_i − n_i)e_i`,
   so `|P_i − n_i e_i| ≤ |n̂_i|(εD_i + 2R_i) + ν_i(D_i + 2R_i)`.
4. `p_i = fl(P_i)` has `|p_i − P_i| ≤ ε|P_i| + 2⁻¹⁰⁷⁵`: relative error at a
   normal result, at most half the least subnormal at a subnormal one.
5. The two sums: `s1 = (p_0 + p_1)(1+σ_1)`, `s = (s1 + p_2)(1+σ_2)`, each
   `|σ| ≤ ε` (a float sum that falls into the subnormals is exact). So
   `|s − Σp_i| ≤ (2ε + ε²)Σ|p_i| ≤ γ_2 Σ|p_i|`, `γ_2 = 2ε/(1−2ε)`, with
   `|p_i| ≤ (1+ε)|P_i| + 2⁻¹⁰⁷⁵`.

Summing, `|s − E| ≤ Σ_i (ε + γ_2(1+ε))|P_i| + 3·2⁻¹⁰⁷⁵(1+γ_2) + Σ_i |P_i − n_i e_i|`.
With `|P_i| ≤ (1+ε)|n̂_i|D_i` the coefficient of `|n̂_i|D_i` is
`ε + ε(1+ε) + 2ε(1+ε)²/(1−2ε) = 4ε + O(ε²) ≤ 5ε`, and
`3·2⁻¹⁰⁷⁵(1+γ_2) ≤ 2⁻¹⁰⁷³`. That is `B`.

`bound ≥ B` because every term of `B` is nonnegative and `plane` forms each one
with upward-rounded operations over upper bounds (`D_i`, `R_i`, `ν_i`). A NaN or
infinite `bound` fails `s ≤ −bound`, so it defers every vertex of the triangle.

The upward steps that form `bound` add at least one least subnormal per step,
which already exceeds the worst underflow error of step 4. Deleting the
`2⁻¹⁰⁷³` floor therefore turns no test red. It stays because the derivation
needs it and the step size is not part of the directed operations' contract.

## Range guard

**The pre-test runs only when every rounded coordinate `v̂` is finite with
`|v̂| ≤ 2²⁵⁰` (`convexGuard`).** A solid with any other coordinate takes every
dot exactly, as the all-exact loop does.

The guard keeps every float of the pre-test finite, which the rounding model
needs: an overflow to an infinity has no relative error.

- `|ê_i| ≤ 2²⁵¹` and `|d_i| ≤ 2²⁵²`.
- The exact normal is a cross product of two such differences, so
  `|n_i| ≤ 2·2⁵⁰²` and `n̂_i` is finite.
- `|P_i| ≤ 2⁵⁰³·2²⁵²`, and `s` is a sum of three of them.

Without the guard a product can round to −∞ while the exact dot is positive,
and `s = −∞` would pass `s ≤ −bound` (`TestConvexFilterRangeGuard`).

The guard has no lower limit. Tiny coordinates make the products underflow,
which steps 2, 4 and 5 bound absolutely, and a normal component that rounds to
zero still charges `ν_i·(D_i + 2R_i)`. At such scales `s` is usually too
small to clear `bound`, so the pairs defer to the exact dot.

## Poll count

**`PlanarConvex` polls once per triangle and vertex pair before deciding the
pair, and returns at the first exact positive sign, as the all-exact loop
does.** A skip is taken only when `E ≤ 0`, which is exactly when the
all-exact loop continues. So both loops visit the same pairs in the same
order, return the same certificate, stop at the same pair, and charge the
same polls. `stepLimit`, `proofbound.WorkBudget` charges and cancellation
points are unchanged (`TestPlanarConvexStopsOnPoll`).

## Tests

All tests live in `internal/pair/planar/planar_convex_test.go`.

- `TestPlanarConvexMatchesExact` compares the certificate and poll count with
  the all-exact loop over convex and non-convex bipyramids and prisms, vertices
  on, one ulp in front of and one ulp behind a facet plane, jittered rings, an
  inexact translation and shear, and power-of-two scalings from `2⁻¹¹⁰⁰` to
  `2¹¹⁰⁰`. It checks every skipped pair's exact sign, and on the regular
  bipyramid that every pair but a triangle's own corners is skipped.
- `TestConvexPlaneBehindIsSound` checks each skip against the exact sign over
  vertices within rounding of their plane, at scales from subnormal products
  to the guard, with float and with inexact coordinates.
- `TestConvexFilterRangeGuard` and `TestConvexPlaneUnderflowedNormal` hold the
  guard and the normal's rounding term on inputs built to defeat each.
- `BenchmarkPlanarConvex` times a bipyramid over an 800-gon and a prism over a
  400-gon.

Each leg below was deleted in turn and turned the named test red:

- The range guard: `TestConvexFilterRangeGuard`.
- `5ε` set to zero: `TestConvexPlaneBehindIsSound`, `TestPlanarConvexMatchesExact`.
- `R_i` left at zero: `TestConvexPlaneBehindIsSound`, `TestPlanarConvexMatchesExact`.
- `ν_i` left at zero: `TestConvexPlaneUnderflowedNormal`.
- The whole bound replaced by zero: all three of the tests above.

The `2⁻¹⁰⁷³` floor is the one term no test can show ("Error bound"). The
`float64` conversions cannot be shown on a target that never fuses, such as
amd64 at the default `GOAMD64=v1`.

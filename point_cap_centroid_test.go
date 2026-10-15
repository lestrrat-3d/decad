package decad

import (
	"context"
	"math"
	"math/big"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// placedCapCentroidBy maps the true centroid by the exact rotation and charges
// only the single rounding of the reported point: the image of the held value
// lies within the reported bound of the reported point, and the bound adds no
// stretch term to the source bound.
func TestPlacedCapCentroidByExactRotation(t *testing.T) {
	source := VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
	for _, tc := range []struct {
		name         string
		center, axis r3.Vec
		theta        float64
	}{
		{"offset oblique axis", r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, 1.1},
		{"X axis through origin", r3.Vec{}, r3.Vec{X: 1}, 2 * math.Pi * 23 / 31},
		{"offset Z axis", r3.Vec{X: 5, Y: 7}, r3.Vec{Z: 1}, math.Pi / 2},
	} {
		t.Run(tc.name, func(t *testing.T) {
			rec, d, err := exactrotation.New(tc.center, tc.axis, units.Radians(tc.theta))
			require.NoError(t, err)
			got, err := placedCapCentroidBy(source, rec.Linear(), rec.Translation())
			require.NoError(t, err)

			// The exact image Q*c + t, computed here from the record's own
			// accessors.
			m, tr := rec.Linear(), rec.Translation()
			c := [3]*big.Rat{ratOfPointPlane(source.Value.X), ratOfPointPlane(source.Value.Y), ratOfPointPlane(source.Value.Z)}
			var image [3]*big.Rat
			dist2 := new(big.Rat)
			for i := range image {
				image[i] = new(big.Rat).Set(tr[i])
				for j := range c {
					image[i].Add(image[i], new(big.Rat).Mul(m[i][j], c[j]))
				}
			}
			held := [3]*big.Rat{ratOfPointPlane(got.Value.X), ratOfPointPlane(got.Value.Y), ratOfPointPlane(got.Value.Z)}
			for i := range image {
				diff := new(big.Rat).Sub(image[i], held[i])
				dist2.Add(dist2, diff.Mul(diff, diff))
			}
			// Only the point's own rounding is charged on top of the source
			// bound, so the reported point sits far inside the reported bound.
			extra := got.Bound.Base() - source.Bound.Base()
			require.GreaterOrEqual(t, extra, 0.0)
			require.Less(t, extra, 1e-12)
			extraRat := ratOfPointPlane(extra)
			require.LessOrEqual(t, dist2.Cmp(extraRat.Mul(extraRat, extraRat)), 0,
				"the rounded point is within the charged rounding of Q*c + t")

			// An independent reading: r3's float placement agrees to rounding.
			want := d.Apply(source.Value)
			require.InDelta(t, want.X, got.Value.X, 1e-12)
			require.InDelta(t, want.Y, got.Value.Y, 1e-12)
			require.InDelta(t, want.Z, got.Value.Z, 1e-12)
		})
	}
}

// placedCapCentroid is placedCapCentroidBy under the placement's own float
// basis and translation, read as exact rationals.
func TestPlacedCapCentroidReadsPlacementAsExactMap(t *testing.T) {
	source := VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
	d, err := r3.RotationAround(r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, units.Radians(1.1))
	require.NoError(t, err)
	linear, err := massmoment.PlacementRotation(d)
	require.NoError(t, err)
	tr := d.Translation()
	want, err := placedCapCentroidBy(source, linear,
		[3]*big.Rat{ratOfPointPlane(tr.X), ratOfPointPlane(tr.Y), ratOfPointPlane(tr.Z)})
	require.NoError(t, err)
	got, err := placedCapCentroid(source, d)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want.Value.X), math.Float64bits(got.Value.X))
	require.Equal(t, math.Float64bits(want.Value.Y), math.Float64bits(got.Value.Y))
	require.Equal(t, math.Float64bits(want.Value.Z), math.Float64bits(got.Value.Z))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
}

// pointPlaneCapCentroidThreeMomentReference freezes the three-coordinate
// integration used before the projected-moment reduction. Keep it independent
// of the production moment loop and dropped-coordinate recovery.
func pointPlaneCapCentroidThreeMomentReference(ctx context.Context, base facetedPayload,
	plane *pointPlaneLimit) (VecMeasurement, [3]*big.Rat, bool, error) {
	if len(base.Verts) < 4 || base.PointSection == nil {
		return VecMeasurement{}, [3]*big.Rat{}, false, nil
	}
	polygon := make([][3]*big.Rat, len(base.Verts)-1)
	for i, far := range base.Verts[1:] {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, [3]*big.Rat{}, false, err
			}
		}
		q, err := pointPlaneMapExact(base.PointSection.Apex, far, plane)
		if err != nil {
			return VecMeasurement{}, [3]*big.Rat{}, false, nil //nolint:nilerr // the frozen reference withholds an optional reading
		}
		polygon[i] = q
	}
	u, v := 0, 2
	if pointPlaneAbs(plane.Nx).Cmp(pointPlaneAbs(plane.Ny)) > 0 {
		u = 1
	}
	twiceArea := new(big.Rat)
	var moments [3]*big.Rat
	for k := range moments {
		moments[k] = new(big.Rat)
	}
	perimeter := 0.0
	for i, a := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, [3]*big.Rat{}, false, err
			}
		}
		b := polygon[(i+1)%len(polygon)]
		perimeter = proofbound.AbsSumUpper(perimeter, pointCapDistanceUpper(a, b))
	}
	anchor := polygon[0]
	for i := 1; i+1 < len(polygon); i++ {
		a, b := polygon[i], polygon[i+1]
		cross := pointPlaneSub(
			pointPlaneMul(pointPlaneSub(a[u], anchor[u]), pointPlaneSub(b[v], anchor[v])),
			pointPlaneMul(pointPlaneSub(b[u], anchor[u]), pointPlaneSub(a[v], anchor[v])))
		twiceArea.Add(twiceArea, cross)
		for k := range moments {
			three := pointPlaneAdd(pointPlaneAdd(anchor[k], a[k]), b[k])
			moments[k].Add(moments[k], pointPlaneMul(three, cross))
		}
	}
	if twiceArea.Sign() == 0 || proofbound.IsNonFinite(perimeter) {
		return VecMeasurement{}, [3]*big.Rat{}, false, nil
	}
	den := pointPlaneMul(big.NewRat(3, 1), twiceArea)
	var center [3]*big.Rat
	for k := range center {
		center[k] = new(big.Rat).Quo(moments[k], den)
	}
	areaLower := proofbound.RatFloatDown(new(big.Rat).Quo(pointPlaneAbs(twiceArea), big.NewRat(2, 1)))
	delta := proofbound.ProductUpper(plane.MapStretch, base.MeshBound)
	tube := proofbound.SectionDisplacementArea(delta, len(polygon), perimeter)
	if areaLower <= tube || proofbound.IsNonFinite(delta) || proofbound.IsNonFinite(tube) {
		return VecMeasurement{}, center, false, nil
	}
	radius := 0.0
	for i, q := range polygon {
		if i%32 == 0 {
			if err := ctx.Err(); err != nil {
				return VecMeasurement{}, center, false, err
			}
		}
		radius = max(radius, pointCapDistanceUpper(q, center))
	}
	floor := math.Nextafter(areaLower-tube, math.Inf(-1))
	if floor <= 0 || proofbound.IsNonFinite(radius) {
		return VecMeasurement{}, center, false, nil
	}
	value, round, err := pointPlaneRound(center)
	if err != nil {
		return VecMeasurement{}, center, false, nil //nolint:nilerr // the frozen reference withholds an optional reading
	}
	shift := proofbound.DivUpper(proofbound.ProductUpper(tube, proofbound.AbsSumUpper(radius, delta)), floor)
	bound := proofbound.AbsSumUpper(shift, round)
	if proofbound.IsNonFinite(bound) {
		return VecMeasurement{}, center, false, nil
	}
	return VecMeasurement{Value: value, Exactness: exactnessOf(bound),
		Bound: units.Millimeters(bound)}, center, true, nil
}

func testPointCapBase(apex r3.Vec, far []r3.Vec, meshBound float64) facetedPayload {
	return facetedPayload{
		Verts: append([]r3.Vec{apex}, far...), PointSection: &pointSectionRecord{Apex: apex},
		MeshBound: meshBound,
	}
}

func requirePointCapThreeMomentMatch(t *testing.T, base facetedPayload, plane *pointPlaneLimit,
	ctx context.Context) {
	t.Helper()
	want, exact, wantOK, wantErr := pointPlaneCapCentroidThreeMomentReference(ctx, base, plane)
	got, gotOK, gotErr := pointPlaneCapCentroid(ctx, base, plane)
	require.ErrorIs(t, gotErr, wantErr)
	require.Equal(t, wantOK, gotOK)
	if !wantOK {
		return
	}
	require.Equal(t, want.Exactness, got.Exactness)
	require.Equal(t, math.Float64bits(want.Value.X), math.Float64bits(got.Value.X))
	require.Equal(t, math.Float64bits(want.Value.Y), math.Float64bits(got.Value.Y))
	require.Equal(t, math.Float64bits(want.Value.Z), math.Float64bits(got.Value.Z))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
	anchor, err := pointPlaneMapExact(base.PointSection.Apex, base.Verts[1], plane)
	require.NoError(t, err)
	keep := 0
	if pointPlaneAbs(plane.Nx).Cmp(pointPlaneAbs(plane.Ny)) > 0 {
		keep = 1
	}
	recovered := pointPlaneCapDroppedCoordinate(exact, anchor, plane, keep)
	require.Zero(t, recovered.Cmp(exact[1-keep]), "the exact dropped centroid coordinate changed")
}

func TestPointPlaneCapCentroidMatchesThreeMoments(t *testing.T) {
	apex := r3.NewVec(1.125, -0.75, 0.375)
	near := math.Nextafter
	cases := []struct {
		name      string
		apex      r3.Vec
		far       []r3.Vec
		nx, ny, d float64
	}{
		{"drop_x_axis_aligned", apex, []r3.Vec{
			r3.NewVec(10, -2, -2), r3.NewVec(10, 2, -2),
			r3.NewVec(10, 2, 2), r3.NewVec(10, -2, 2)}, 1, 0, 4},
		{"drop_y_axis_aligned", apex, []r3.Vec{
			r3.NewVec(-2, 10, -2), r3.NewVec(2, 10, -2),
			r3.NewVec(2, 10, 2), r3.NewVec(-2, 10, 2)}, 0, 1, 4},
		{"drop_x_oblique", apex, []r3.Vec{
			r3.NewVec(8, -2, -2), r3.NewVec(10, -1, -2),
			r3.NewVec(9, 2, 2), r3.NewVec(7, 1, 2)}, 3, 2, 6},
		{"drop_y_oblique", apex, []r3.Vec{
			r3.NewVec(5, 8, -2), r3.NewVec(7, 7, -2),
			r3.NewVec(8, 10, 2), r3.NewVec(4, 11, 2)}, 2, 3, 6},
		{"large_projective_denominator_x", apex, []r3.Vec{
			r3.NewVec(near(8, 9), 1.0/3, -2), r3.NewVec(9, near(3, 4), -1),
			r3.NewVec(11, 4, 2), r3.NewVec(7, 2, 3)}, near(0.7, 1), 0.6, 2.5},
		{"large_projective_denominator_y", apex, []r3.Vec{
			r3.NewVec(1.0/3, near(8, 9), -2), r3.NewVec(near(3, 4), 9, -1),
			r3.NewVec(4, 11, 2), r3.NewVec(2, 7, 3)}, 0.6, near(0.7, 1), 2.5},
		{"halfway_rounding", r3.Vec{}, []r3.Vec{
			r3.NewVec(10, 1, 0), r3.NewVec(10, near(1, 2), 0),
			r3.NewVec(10, near(1, 2), 2), r3.NewVec(10, 1, 2)}, 1, 0, 5},
	}
	for _, tc := range cases {
		for _, reverse := range []bool{false, true} {
			name := tc.name + "/forward"
			far := append([]r3.Vec(nil), tc.far...)
			if reverse {
				name = tc.name + "/reverse"
				for i, j := 0, len(far)-1; i < j; i, j = i+1, j-1 {
					far[i], far[j] = far[j], far[i]
				}
			}
			t.Run(name, func(t *testing.T) {
				plane := &pointPlaneLimit{Nx: ratOfPointPlane(tc.nx), Ny: ratOfPointPlane(tc.ny),
					D: ratOfPointPlane(tc.d), MapStretch: 1}
				base := testPointCapBase(tc.apex, far, 0)
				requirePointCapThreeMomentMatch(t, base, plane, t.Context())
				if tc.name == "halfway_rounding" {
					_, exact, proved, err := pointPlaneCapCentroidThreeMomentReference(t.Context(), base, plane)
					require.NoError(t, err)
					require.True(t, proved)
					lo := new(big.Rat).SetFloat64(0.5)
					hi := new(big.Rat).SetFloat64(math.Nextafter(0.5, 1))
					midpoint := new(big.Rat).Quo(pointPlaneAdd(lo, hi), big.NewRat(2, 1))
					require.Zero(t, exact[1].Cmp(midpoint))
				}
			})
		}
	}
	t.Run("large_denominator_growth", func(t *testing.T) {
		far := make([]r3.Vec, 48)
		for i := range far {
			angle := 2 * math.Pi * float64(i) / float64(len(far))
			far[i] = r3.NewVec(9+math.Sin(3*angle)/8, 3+math.Cos(angle), math.Sin(angle))
		}
		plane := &pointPlaneLimit{Nx: ratOfPointPlane(near(0.7, 1)), Ny: ratOfPointPlane(0.6),
			D: ratOfPointPlane(2.5), MapStretch: 1}
		base := testPointCapBase(apex, far, 0)
		requirePointCapThreeMomentMatch(t, base, plane, t.Context())
		_, exact, proved, err := pointPlaneCapCentroidThreeMomentReference(t.Context(), base, plane)
		require.NoError(t, err)
		require.True(t, proved)
		require.Greater(t, exact[1].Denom().BitLen(), 512)
	})
}

type pointCapCancelAfterContext struct {
	calls, cancelAt int
}

func (*pointCapCancelAfterContext) Deadline() (time.Time, bool) { return time.Time{}, false }

func (*pointCapCancelAfterContext) Done() <-chan struct{} { return nil }

func (*pointCapCancelAfterContext) Value(any) any { return nil }

func (c *pointCapCancelAfterContext) Err() error {
	c.calls++
	if c.calls >= c.cancelAt {
		return context.Canceled
	}
	return nil
}

func TestPointPlaneCapCentroidRefusalsMatchThreeMoments(t *testing.T) {
	good := []r3.Vec{r3.NewVec(10, -2, -2), r3.NewVec(10, 2, -2),
		r3.NewVec(10, 2, 2), r3.NewVec(10, -2, 2)}
	plane := &pointPlaneLimit{Nx: big.NewRat(1, 1), Ny: new(big.Rat),
		D: big.NewRat(5, 1), MapStretch: 1}
	t.Run("too_few_vertices", func(t *testing.T) {
		requirePointCapThreeMomentMatch(t, testPointCapBase(r3.Vec{}, good[:2], 0), plane, t.Context())
	})
	t.Run("missing_source", func(t *testing.T) {
		base := testPointCapBase(r3.Vec{}, good, 0)
		base.PointSection = nil
		requirePointCapThreeMomentMatch(t, base, plane, t.Context())
	})
	t.Run("ray_misses_plane", func(t *testing.T) {
		bad := append([]r3.Vec(nil), good...)
		bad[0].X = -10
		requirePointCapThreeMomentMatch(t, testPointCapBase(r3.Vec{}, bad, 0), plane, t.Context())
	})
	t.Run("zero_projected_area", func(t *testing.T) {
		flat := []r3.Vec{r3.NewVec(10, 0, 0), r3.NewVec(10, 1, 0),
			r3.NewVec(10, 2, 0), r3.NewVec(10, 3, 0)}
		requirePointCapThreeMomentMatch(t, testPointCapBase(r3.Vec{}, flat, 0), plane, t.Context())
	})
	t.Run("area_floor", func(t *testing.T) {
		requirePointCapThreeMomentMatch(t, testPointCapBase(r3.Vec{}, good, 100), plane, t.Context())
	})
	t.Run("canceled", func(t *testing.T) {
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		requirePointCapThreeMomentMatch(t, testPointCapBase(r3.Vec{}, good, 0), plane, ctx)
	})
	t.Run("canceled_before_radius", func(t *testing.T) {
		far := make([]r3.Vec, 48)
		for i := range far {
			angle := 2 * math.Pi * float64(i) / float64(len(far))
			far[i] = r3.NewVec(10, 2*math.Cos(angle), 2*math.Sin(angle))
		}
		base := testPointCapBase(r3.Vec{}, far, 0)
		oldCtx := &pointCapCancelAfterContext{cancelAt: 5}
		_, _, oldOK, oldErr := pointPlaneCapCentroidThreeMomentReference(oldCtx, base, plane)
		newCtx := &pointCapCancelAfterContext{cancelAt: 5}
		_, newOK, newErr := pointPlaneCapCentroid(newCtx, base, plane)
		require.ErrorIs(t, oldErr, context.Canceled)
		require.ErrorIs(t, newErr, context.Canceled)
		require.False(t, oldOK)
		require.False(t, newOK)
		require.Equal(t, oldCtx.calls, newCtx.calls)
	})
}

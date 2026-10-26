package facetedrecord_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/exactrotation"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/measurement"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/surfacenormal"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// PlacedCapCentroidBy maps the true centroid by the exact rotation and charges
// only the single rounding of the reported point: the image of the held value
// lies within the reported bound of the reported point, and the bound adds no
// stretch term to the source bound.
func TestPlacedCapCentroidByExactRotation(t *testing.T) {
	source := measurement.VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
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
			got, err := facetedrecord.PlacedCapCentroidBy(source, rec.Linear(), rec.Translation())
			require.NoError(t, err)

			// The exact image Q*c + t, computed here from the record's own
			// accessors.
			m, tr := rec.Linear(), rec.Translation()
			c := [3]*big.Rat{proofarith.FloatRat(source.Value.X), proofarith.FloatRat(source.Value.Y),
				proofarith.FloatRat(source.Value.Z)}
			var image [3]*big.Rat
			dist2 := new(big.Rat)
			for i := range image {
				image[i] = new(big.Rat).Set(tr[i])
				for j := range c {
					image[i].Add(image[i], new(big.Rat).Mul(m[i][j], c[j]))
				}
			}
			held := [3]*big.Rat{proofarith.FloatRat(got.Value.X), proofarith.FloatRat(got.Value.Y),
				proofarith.FloatRat(got.Value.Z)}
			for i := range image {
				diff := new(big.Rat).Sub(image[i], held[i])
				dist2.Add(dist2, diff.Mul(diff, diff))
			}
			// Only the point's own rounding is charged on top of the source
			// bound, so the reported point sits far inside the reported bound.
			extra := got.Bound.Base() - source.Bound.Base()
			require.GreaterOrEqual(t, extra, 0.0)
			require.Less(t, extra, 1e-12)
			extraRat := proofarith.FloatRat(extra)
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

// PlacedCapCentroid is PlacedCapCentroidBy under the placement's own float
// basis and translation, read as exact rationals.
func TestPlacedCapCentroidReadsPlacementAsExactMap(t *testing.T) {
	source := measurement.VecMeasurement{Value: r3.Vec{X: 1.5, Y: -2, Z: 0.25}, Bound: units.Millimeters(0.001)}
	d, err := r3.RotationAround(r3.Vec{X: 2, Y: -1, Z: 3}, r3.Vec{X: 1, Y: 2, Z: 2}, units.Radians(1.1))
	require.NoError(t, err)
	linear, err := massmoment.PlacementRotation(d)
	require.NoError(t, err)
	tr := d.Translation()
	want, err := facetedrecord.PlacedCapCentroidBy(source, linear,
		[3]*big.Rat{proofarith.FloatRat(tr.X), proofarith.FloatRat(tr.Y), proofarith.FloatRat(tr.Z)})
	require.NoError(t, err)
	got, err := facetedrecord.PlacedCapCentroid(source, d)
	require.NoError(t, err)
	require.Equal(t, math.Float64bits(want.Value.X), math.Float64bits(got.Value.X))
	require.Equal(t, math.Float64bits(want.Value.Y), math.Float64bits(got.Value.Y))
	require.Equal(t, math.Float64bits(want.Value.Z), math.Float64bits(got.Value.Z))
	require.Equal(t, math.Float64bits(want.Bound.Base()), math.Float64bits(got.Bound.Base()))
}

func exactGroupFixture(t *testing.T, degrees float64) (*exactrotation.Rotation, r3.Transform, []r3.Vec) {
	t.Helper()
	center, axis := r3.NewVec(20, 10, 0), r3.NewVec(0, 0, 1)
	q, delta, err := exactrotation.New(center, axis, units.Degrees(degrees))
	require.NoError(t, err)
	return q, delta, []r3.Vec{r3.NewVec(0, 0, 0), r3.NewVec(40, 20, 20)}
}

func TestExactGroupPlacedMapsCentroidAndDropsTags(t *testing.T) {
	q, d, err := exactrotation.New(r3.Vec{}, r3.Vec{Z: 1}, units.Degrees(90))
	require.NoError(t, err)
	origin := provenance.Ref{Producer: 1, Role: provenance.RoleCapStart}
	g := facetedrecord.Group{
		Origins: []provenance.Ref{origin}, HasCentroid: true, Planar: true, Reversed: true,
		Centroid:    measurement.VecMeasurement{Value: r3.NewVec(2, 0, 5), Exactness: measurement.Exact},
		Surface:     surfacegeom.Plane{},
		NormalBound: 3,
	}
	require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, d, nil, 0))
	require.Nil(t, g.Surface, "a non-retained tag describes the float placement")
	require.Nil(t, g.Denoted)
	require.Zero(t, g.NormalBound)
	require.False(t, g.Retained)
	require.True(t, g.Planar)
	require.True(t, g.Reversed)
	require.Equal(t, []provenance.Ref{origin}, g.Origins)
	require.InDelta(t, 0, g.Centroid.Value.X, 1e-12)
	require.InDelta(t, 2, g.Centroid.Value.Y, 1e-12)
	require.InDelta(t, 5, g.Centroid.Value.Z, 1e-12)
	// The reading is exactly Q*c: Q is orthogonal, so the stretch charge is 0.
	lin := q.Linear()
	exactY := new(big.Rat).Mul(lin[1][0], big.NewRat(2, 1))
	got := new(big.Rat)
	got.SetFloat64(g.Centroid.Value.Y)
	delta := new(big.Rat).Sub(got, exactY)
	delta.Abs(delta)
	bound := new(big.Rat)
	require.NotNil(t, bound.SetFloat64(g.Centroid.Bound.Base()))
	require.LessOrEqual(t, delta.Cmp(bound), 0, "reading strays from exact Q*c beyond its bound")
}

func TestExactGroupPlacedKeepsRetainedTags(t *testing.T) {
	t.Parallel()
	q, delta, verts := exactGroupFixture(t, 20)
	frame, err := r3.NewFrame(r3.NewVec(0, 0, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	plane := surfacegeom.Plane{Frame: frame}
	cyl := surfacegeom.Cylinder{Origin: r3.NewVec(20, 0, 10), Axis: r3.NewVec(0, 1, 0), Radius: units.Millimeters(4)}

	t.Run("a retained plane takes the float image and the 8a+P charge against Q", func(t *testing.T) {
		t.Parallel()
		g := facetedrecord.Group{Surface: plane, Retained: true, NormalBound: 1e-12}
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		moved, ok := g.Surface.(surfacegeom.Plane)
		require.True(t, ok)
		require.True(t, g.Retained)
		// Recompute a: the exact image Q*U against the moved U.
		lin := q.Linear()
		a := 0.0
		for i, want := range [3]float64{moved.Frame.U().X, moved.Frame.U().Y, moved.Frame.U().Z} {
			exact := new(big.Rat).Set(lin[i][0])
			diff := new(big.Rat).Sub(exact, new(big.Rat).SetFloat64(want))
			f, _ := diff.Abs(diff).Float64()
			a = max(a, f)
		}
		require.Positive(t, a, "a 20 degree image rounds")
		require.GreaterOrEqual(t, g.NormalBound, 1e-12+8*a)
		require.Less(t, g.NormalBound, 1e-9)
		require.InDelta(t, math.Cos(20*math.Pi/180), moved.Frame.U().X, 1e-12)
		require.InDelta(t, math.Sin(20*math.Pi/180), moved.Frame.U().Y, 1e-12)
	})

	t.Run("a retained cylinder takes the float image and a finite bound", func(t *testing.T) {
		t.Parallel()
		g := facetedrecord.Group{Surface: cyl, Retained: true}
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		moved, ok := g.Surface.(surfacegeom.Cylinder)
		require.True(t, ok)
		require.InDelta(t, -math.Sin(20*math.Pi/180), moved.Axis.X, 1e-12)
		require.InDelta(t, math.Cos(20*math.Pi/180), moved.Axis.Y, 1e-12)
		require.Positive(t, g.NormalBound)
		require.Less(t, g.NormalBound, 1e-9)
	})

	t.Run("a retained cone keeps the trivial bound", func(t *testing.T) {
		t.Parallel()
		cone := surfacegeom.Cone{Axis: r3.NewVec(0, 0, 1), Radius: units.Millimeters(1), HalfAngle: units.Degrees(10)}
		g := facetedrecord.Group{Surface: cone, Retained: true}
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.IsType(t, surfacegeom.Cone{}, g.Surface)
		require.InDelta(t, 2, g.NormalBound, 0)
	})

	t.Run("a retained tag with a denoted patch is dropped", func(t *testing.T) {
		t.Parallel()
		g := facetedrecord.Group{Surface: plane, Retained: true, Denoted: &surfacenormal.Revolved{}}
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Nil(t, g.Surface)
		require.Nil(t, g.Denoted)
		require.False(t, g.Retained)
	})

	t.Run("a far vertex raises the curved bound with the displacement", func(t *testing.T) {
		t.Parallel()
		g := facetedrecord.Group{Surface: cyl, Retained: true}
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, []r3.Vec{r3.NewVec(1e16, 0, 0)}, 1e-9))
		require.Greater(t, g.NormalBound, 0.1)
		require.LessOrEqual(t, g.NormalBound, 2.0)
	})
}

// A plane whose exact normal the rotation fixes is its own image, so an exact
// placement keeps its tag, CapRecord and axial bound with bound 0. Each leg of
// the check refuses on its own.
func TestExactGroupPlacedKeepsFixedPlane(t *testing.T) {
	t.Parallel()
	q, delta, verts := exactGroupFixture(t, 20)
	frame, err := r3.NewFrame(r3.NewVec(3, 4, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 1, 0))
	require.NoError(t, err)
	tilted, err := r3.NewFrame(r3.NewVec(3, 4, 20), r3.NewVec(1, 0, 0), r3.NewVec(0, 0.6, 0.8))
	require.NoError(t, err)
	record := &topo.CapRecord{Frame: frame, Xform: r3.Identity(), Z: 0}
	group := func(tag r3.Frame, rec *topo.CapRecord) facetedrecord.Group {
		return facetedrecord.Group{
			Surface: surfacegeom.Plane{Frame: tag}, Retained: true,
			CapRecord: rec, AxialDelta: 1e-9, HasAxialDelta: rec != nil,
		}
	}

	t.Run("the tag, record and axial bound survive with bound 0", func(t *testing.T) {
		t.Parallel()
		g := group(frame, record)
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Zero(t, g.NormalBound)
		require.True(t, g.Retained)
		require.Equal(t, surfacegeom.Plane{Frame: frame}, g.Surface)
		require.Same(t, record, g.CapRecord)
		require.True(t, g.HasAxialDelta)
		require.Equal(t, 1e-9, g.AxialDelta)
	})

	t.Run("a tag without a record keeps bound 0 and states no record", func(t *testing.T) {
		t.Parallel()
		g := group(frame, nil)
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Zero(t, g.NormalBound)
		require.Nil(t, g.CapRecord)
		require.False(t, g.HasAxialDelta)
	})

	t.Run("a tilted tag is charged and loses its record", func(t *testing.T) {
		t.Parallel()
		g := group(tilted, record)
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Positive(t, g.NormalBound)
		require.Nil(t, g.CapRecord)
		require.False(t, g.HasAxialDelta)
	})

	t.Run("a flat tag over a tilted record loses the record and keeps no bound 0", func(t *testing.T) {
		t.Parallel()
		g := group(frame, &topo.CapRecord{Frame: tilted, Xform: r3.Identity(), Z: 0})
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Positive(t, g.NormalBound)
		require.Nil(t, g.CapRecord)
	})

	t.Run("a tag with its own bound takes the charged path", func(t *testing.T) {
		t.Parallel()
		g := group(frame, record)
		g.NormalBound = 1e-12
		require.NoError(t, facetedrecord.ExactGroupPlaced(&g, q, delta, verts, 1e-9))
		require.Positive(t, g.NormalBound)
		require.Nil(t, g.CapRecord)
	})
}

func TestPlaneFixedBy(t *testing.T) {
	t.Parallel()
	q, _, _ := exactGroupFixture(t, 20)
	linear, trans := q.Linear(), q.Translation()
	rat := func(a, b, c int64) [3]*big.Rat {
		return [3]*big.Rat{big.NewRat(a, 1), big.NewRat(b, 1), big.NewRat(c, 1)}
	}
	require.True(t, facetedrecord.PlaneFixedBy(linear, trans, rat(0, 0, 1)))
	require.True(t, facetedrecord.PlaneFixedBy(linear, trans, rat(0, 0, -3)), "the length of m is free")
	require.False(t, facetedrecord.PlaneFixedBy(linear, trans, rat(1, 0, 0)), "an in-plane direction turns")
	require.False(t, facetedrecord.PlaneFixedBy(linear, trans, rat(0, 0, 0)), "no direction")

	t.Run("a translation along m moves the plane", func(t *testing.T) {
		t.Parallel()
		require.False(t, facetedrecord.PlaneFixedBy(linear, rat(0, 0, 1), rat(0, 0, 1)))
	})

	t.Run("a map whose transpose does not fix m refuses", func(t *testing.T) {
		t.Parallel()
		shear := linear
		shear[2][0] = big.NewRat(1, 2)
		require.False(t, facetedrecord.PlaneFixedBy(shear, trans, rat(0, 0, 1)))
	})

	t.Run("a map that does not fix m refuses", func(t *testing.T) {
		t.Parallel()
		shear := linear
		shear[0][2] = big.NewRat(1, 2)
		require.False(t, facetedrecord.PlaneFixedBy(shear, trans, rat(0, 0, 1)))
	})
}

package apitest_test

import (
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// The fixtures below union prisms whose outlines touch at one point without
// crossing: a circle tangent to a line or to another circle, or a triangle's
// apex on a square's side. Sketch's arrangement cuts neither, so the touching
// loops come back whole. A prism group over them, or a stacked interface whose
// hole touches the outer wall, would hold two loops meeting at that point,
// which no cap mesh can separate (agent-docs/general-boolean-design.md §3 A5,
// agent-docs/stacked-prism-design.md §2.2).

// polygonPrismIn extrudes a polygon on XY into doc by 10 along +z.
func polygonPrismIn(t *testing.T, doc *decad.Document, pts ...[2]float64) *decad.Body {
	t.Helper()
	s, p := polygonSketch(t, pts)
	body, err := decad.Extrude(t.Context(), doc, s, p, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

// touchSquare is the 5×5×10 square at the origin, 250 mm³.
func touchSquare(t *testing.T, doc *decad.Document) *decad.Body {
	return polygonPrismIn(t, doc, [2]float64{0, 0}, [2]float64{5, 0}, [2]float64{5, 5}, [2]float64{0, 5})
}

// touchTriangle has its apex (5, 2) on touchSquare's x = 5 side; its area is
// 5.5 mm², 55 mm³. No two of its lines share a carrier with the square's.
func touchTriangle(t *testing.T, doc *decad.Document) *decad.Body {
	return polygonPrismIn(t, doc, [2]float64{5, 2}, [2]float64{9, 1}, [2]float64{8, 4})
}

// requireMeshedUnion asserts a mesh-path result whose volume bound covers the
// closed form and whose mesh passes VerifyAll.
func requireMeshedUnion(t *testing.T, got *decad.Body, want float64) {
	t.Helper()
	require.True(t, tookMeshPath(got))
	volume, err := got.Volume()
	require.NoError(t, err)
	require.Equal(t, decad.Approximate, volume.Exactness)
	require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-want), boundMM3(t, volume))
	mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.5))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
}

// TestUnionTangentContactTakesMeshPath unions operands that touch at one
// point. None builds a prism group or a stacked interface over the touching
// loops; each takes the mesh path, which meshes and covers the closed form.
func TestUnionTangentContactTakesMeshPath(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		union  func(t *testing.T, doc *decad.Document) (*decad.Body, error)
		volume float64
	}{
		{
			// r = 2 at (12, 0) touches the plate's x = 10 side at (10, 0).
			name: "circle tangent to the plate's side",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 12, 2, 0, 10))
			},
			volume: 4000 + 40*math.Pi,
		},
		{
			name: "two tangent circles",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), circleBodyAtZ(t, doc, 0, 2, 0, 10), circleBodyAtZ(t, doc, 5, 3, 0, 10))
			},
			volume: 130 * math.Pi,
		},
		{
			// A prism group operand: the plate and a far block, then the
			// tangent circle.
			name: "circle tangent to a group's plate",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				group, err := decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), boxBody(t, doc, 30, 0, 35, 5, 10))
				require.NoError(t, err)
				require.False(t, tookMeshPath(group))
				return decad.Union(t.Context(), group, circleBodyAtZ(t, doc, 12, 2, 0, 10))
			},
			volume: 4250 + 40*math.Pi,
		},
		{
			name: "triangle apex on the square's side",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), touchSquare(t, doc), touchTriangle(t, doc))
			},
			volume: 305,
		},
		{
			// A boss on the plate's top whose wall touches the plate's x = 10
			// side from inside: the top face would hold the boss outline as a
			// hole touching its outer.
			name: "stacked boss inside the square edge",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 8, 2, 10, 4))
			},
			volume: 4000 + 16*math.Pi,
		},
		{
			name: "stacked boss inside the round edge",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), circleBodyAtZ(t, doc, 0, 10, 0, 10), circleBodyAtZ(t, doc, 8, 2, 10, 4))
			},
			volume: 1016 * math.Pi,
		},
		{
			name: "boss through the plate's top inside its edge",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 8, 2, 5, 9))
			},
			volume: 4000 + 16*math.Pi,
		},
		{
			// Three bodies: no route takes the set, so they join two at a
			// time and the tangent pair takes the mesh path.
			name: "three circles, two tangent",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), circleBodyAtZ(t, doc, 0, 2, 0, 10), circleBodyAtZ(t, doc, 4, 2, 0, 10),
					circleBodyAtZ(t, doc, 20, 2, 0, 10))
			},
			volume: 120 * math.Pi,
		},
		{
			// Line-only prisms reach the one-scene many-operand Union, whose
			// loops close as three but touch at the apex.
			name: "square, triangle apex on its side and a far square",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				far := polygonPrismIn(t, doc, [2]float64{20, 1}, [2]float64{24, 1}, [2]float64{24, 4}, [2]float64{20, 4})
				return decad.Union(t.Context(), touchSquare(t, doc), touchTriangle(t, doc), far)
			},
			volume: 425,
		},
		{
			// The triangle's corner (5, 5) is the square's corner: the
			// one-scene boundary does not chain into loops, so the bodies
			// join two at a time. The triangle's area is 7.5 mm².
			name: "square, triangle on its corner and a far square",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				corner := polygonPrismIn(t, doc, [2]float64{5, 5}, [2]float64{9, 6}, [2]float64{6, 9})
				far := polygonPrismIn(t, doc, [2]float64{20, 1}, [2]float64{24, 1}, [2]float64{24, 4}, [2]float64{20, 4})
				return decad.Union(t.Context(), touchSquare(t, doc), corner, far)
			},
			volume: 445,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := tc.union(t, decad.New())
			require.NoError(t, err)
			requireMeshedUnion(t, got, tc.volume)
		})
	}
}

// TestUnionNearStraightVertexRefuses unions the square with a quadrilateral
// whose vertex (5, 2) lies on the square's x = 5 side and whose two edges
// there leave that side by about 2.5e-10 rad, inside sketch's own parallel
// band, so sketch's line query reports no common point for either edge.
// Sketch's arrangement reports the region invalid, so the pair is refused
// before any contact reading runs (agent-docs/stacked-prism-design.md §2.2).
func TestUnionNearStraightVertexRefuses(t *testing.T) {
	t.Parallel()
	nearStraight := func(t *testing.T, doc *decad.Document) *decad.Body {
		return polygonPrismIn(t, doc, [2]float64{5, 2}, [2]float64{5 + 5e-10, 4}, [2]float64{9, 3}, [2]float64{5 + 5e-10, 0})
	}
	cases := []struct {
		name  string
		union func(t *testing.T, doc *decad.Document) (*decad.Body, error)
	}{
		{name: "two bodies", union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
			return decad.Union(t.Context(), touchSquare(t, doc), nearStraight(t, doc))
		}},
		{name: "three bodies", union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
			far := polygonPrismIn(t, doc, [2]float64{20, 1}, [2]float64{24, 1}, [2]float64{24, 4}, [2]float64{20, 4})
			return decad.Union(t.Context(), touchSquare(t, doc), nearStraight(t, doc), far)
		}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			_, err := tc.union(t, doc)
			require.ErrorIs(t, err, decad.ErrUnsupported)
			require.ErrorContains(t, err, "arrangement reports an invalid region")
		})
	}
}

// TestPatternedTangentCopies patterns a disc whose copies touch. Patterned
// does not group them; its Union fallback takes the mesh path.
func TestPatternedTangentCopies(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name string
		x    float64
		spec decad.PatternSpec
	}{
		{name: "linear", x: 0, spec: decad.LinearPattern{Dir: patternX, Step: units.Millimeters(4), Count: 2}},
		{name: "circular", x: 2, spec: decad.CircularPattern{Axis: patternZ, Count: 2}},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New()
			got, err := decad.Patterned(t.Context(), circleBodyAtZ(t, doc, tc.x, 2, 0, 10), tc.spec)
			require.NoError(t, err)
			requireMeshedUnion(t, got, 80*math.Pi)
		})
	}
}

// TestUnionApartOrNestedStaysAnalytic keeps the analytic routes a touch
// does not reach: a circle tangent inside the other operand at one level is
// that operand, regions with a gap form a prism group, and a boss clear of
// the plate's edge is a stacked prism.
func TestUnionApartOrNestedStaysAnalytic(t *testing.T) {
	t.Parallel()
	cases := []struct {
		name   string
		union  func(t *testing.T, doc *decad.Document) (*decad.Body, error)
		volume float64
		exact  bool
	}{
		{
			name: "circle tangent inside the plate",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 8, 2, 0, 10))
			},
			volume: 4000,
			exact:  true,
		},
		{
			name: "circle tangent inside a circle",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), circleBodyAtZ(t, doc, 0, 10, 0, 10), circleBodyAtZ(t, doc, 8, 2, 0, 10))
			},
			volume: 1000 * math.Pi,
		},
		{
			name: "circles with a gap",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), circleBodyAtZ(t, doc, 0, 2, 0, 10), circleBodyAtZ(t, doc, 5, 2, 0, 10))
			},
			volume: 80 * math.Pi,
		},
		{
			name: "triangle clear of the square",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				apart := polygonPrismIn(t, doc, [2]float64{6, 2}, [2]float64{10, 1}, [2]float64{9, 4})
				return decad.Union(t.Context(), touchSquare(t, doc), apart)
			},
			volume: 305,
			exact:  true,
		},
		{
			name: "boss clear of the edge",
			union: func(t *testing.T, doc *decad.Document) (*decad.Body, error) {
				return decad.Union(t.Context(), boxBody(t, doc, -10, -10, 10, 10, 10), circleBodyAtZ(t, doc, 5, 2, 10, 4))
			},
			volume: 4000 + 16*math.Pi,
		},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			got, err := tc.union(t, decad.New())
			require.NoError(t, err)
			require.False(t, tookMeshPath(got))
			volume, err := got.Volume()
			require.NoError(t, err)
			if tc.exact {
				require.Equal(t, decad.Exact, volume.Exactness)
				require.Equal(t, tc.volume, volumeMM(t, volume))
			} else {
				require.LessOrEqual(t, math.Abs(volumeMM(t, volume)-tc.volume), boundMM3(t, volume))
			}
			mesh, err := got.Tessellate(t.Context(), units.Millimeters(0.5))
			require.NoError(t, err)
			require.True(t, mesh.BoundaryVerified())
			require.True(t, mesh.VolumeVerified())
		})
	}
}

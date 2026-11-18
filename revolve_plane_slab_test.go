package decad

import (
	"context"
	"errors"
	"math"
	"regexp"
	"strconv"
	"testing"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// revolveDisc is a disc revolved about the world Z axis from the meridian
// rectangle r in [0, radius], z in [z0, z1].
func revolveDisc(ctx context.Context, t *testing.T, doc *Document, radius, z0, z1 float64) *Body {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{}, r3.Vec{Z: 1}, r3.Vec{X: 1})
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	pts := [][2]float64{{z0, 0}, {z0, radius}, {z1, radius}, {z1, 0}}
	points := make([]*sketch.Point, len(pts))
	for i, p := range pts {
		points[i] = s.CreatePoint(p[0], p[1])
		s.Fix(points[i])
	}
	for i := range points {
		s.CreateLine(points[i], points[(i+1)%len(points)])
	}
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	body, err := Revolve(ctx, doc, s, s.Profiles()[0], ConstructionAxis{Origin: r3.Vec{}, Dir: r3.Vec{Z: 1}},
		FullRevolution{})
	require.NoError(t, err)
	return body
}

// axialTool extrudes, from the plane z = z0 along +Z by length, the circles
// of the given radii about (x, y): one circle is a rod, two an annulus.
func axialTool(ctx context.Context, t *testing.T, doc *Document, x, y, z0, length float64, radii ...float64) *Body {
	t.Helper()
	f, err := r3.NewFrame(r3.Vec{Z: z0}, r3.Vec{X: 1}, r3.Vec{Y: 1})
	require.NoError(t, err)
	w := sketch.NewWorld()
	plane, err := w.CreatePlaneFromFrame(f)
	require.NoError(t, err)
	s, err := w.CreateSketch(plane)
	require.NoError(t, err)
	c := s.CreatePoint(x, y)
	s.Fix(c)
	for _, r := range radii {
		s.AddConstraint(sketch.NewRadius(s.CreateCircle(c, r), r))
	}
	_, err = s.Solve(ctx)
	require.NoError(t, err)
	var pr *sketch.Profile
	for _, p := range s.Profiles() {
		if p.Valid && len(p.Holes) == len(radii)-1 {
			pr = p
		}
	}
	require.NotNil(t, pr)
	body, err := Extrude(ctx, doc, s, pr, Distance{D: units.Millimeters(length), Dir: Along})
	require.NoError(t, err)
	return body
}

// revolvePlaneScene is a revolved disc cut by an axial bore, optionally an
// annular groove into its top face, then small axial holes on a circle.
type revolvePlaneScene struct {
	radius, z0, z1   float64
	bore             float64
	groove           [2]float64 // inner and outer radius; zero for none
	grooveDepth      float64
	holes            int
	holeR, holeAt    float64
	holeFirst, holeD float64 // first angle and step, degrees
	copyEach         bool    // end the Boolean chain before every hole
}

// holeCentre is hole i's centre on the holes' circle.
func (sc revolvePlaneScene) holeCentre(i int) (float64, float64) {
	a := (sc.holeFirst + sc.holeD*float64(i)) * math.Pi / 180
	return sc.holeAt * math.Cos(a), sc.holeAt * math.Sin(a)
}

// build runs the scene's Booleans in order and returns the result of the
// last one, or the first error with the step it came from.
func (sc revolvePlaneScene) build(ctx context.Context, t *testing.T) (*Body, string, error) {
	t.Helper()
	doc := New()
	h := sc.z1 - sc.z0
	body, err := Cut(ctx, revolveDisc(ctx, t, doc, sc.radius, sc.z0, sc.z1),
		axialTool(ctx, t, doc, 0, 0, sc.z0-1, h+2, sc.bore))
	if err != nil {
		return nil, "bore", err
	}
	if sc.groove[1] > 0 {
		groove := axialTool(ctx, t, doc, 0, 0, sc.z1-sc.grooveDepth, sc.grooveDepth+1, sc.groove[1], sc.groove[0])
		if body, err = Cut(ctx, body, groove); err != nil {
			return nil, "groove", err
		}
	}
	for i := range sc.holes {
		if sc.copyEach {
			body, err = Copy(ctx, body)
			require.NoError(t, err)
		}
		x, y := sc.holeCentre(i)
		if body, err = Cut(ctx, body, axialTool(ctx, t, doc, x, y, sc.z0-1, h+2, sc.holeR)); err != nil {
			return nil, "hole " + strconv.Itoa(i+1), err
		}
	}
	return body, "", nil
}

// requireChainDepthRefusal asserts err is the held-mesh chain-depth refusal
// and returns the bound it quotes and the pair's chord tolerance.
func requireChainDepthRefusal(t *testing.T, err error) (float64, float64) {
	t.Helper()
	require.ErrorIs(t, err, ErrUnsupported)
	var be *BooleanError
	require.False(t, errors.As(err, &be), "the chain-depth refusal is a plain ErrUnsupported")
	m := regexp.MustCompile(`within a bound of (\S+) mm, coarser than this pair's chord tolerance (\S+) mm`).
		FindStringSubmatch(err.Error())
	require.Len(t, m, 3, "the refusal quotes the touched bound and the pair's chord tolerance: %v", err)
	touched, perr := strconv.ParseFloat(m[1], 64)
	require.NoError(t, perr)
	tol, perr := strconv.ParseFloat(m[2], 64)
	require.NoError(t, perr)
	require.Greater(t, touched, tol)
	return touched, tol
}

// TestRevolvePlaneSlab checks retention §11's slab on the planar faces of a
// revolve whose tag is exactly the plane its record denotes
// (booleaneval.retainedPlaneOf, stitchflux.TagIsDenoted). A bore leaves rim
// vertices whose bound spreads over the planar faces' long fan facets, and a
// later small hole that lands on such a facet is refused by the held-mesh gate
// unless the facet's true piece is known to lie in its own plane. Each case
// builds, and the full-displacement control (booleaneval.WithoutRetainedPlaneSlab)
// refuses the same scene with the chain-depth refusal. The built body's volume
// matches the exact solid, and every vertex on a hole's rim lies within its
// published bound of the exact circle.
func TestRevolvePlaneSlab(t *testing.T) {
	t.Parallel()

	for _, tc := range []struct {
		name string
		sc   revolvePlaneScene
	}{
		{"revolve, bore, groove and four holes", revolvePlaneScene{
			radius: 20, z0: -3.8, z1: 0, bore: 3.25, groove: [2]float64{12, 15}, grooveDepth: 1.9,
			holes: 4, holeR: 0.75, holeAt: 8, holeFirst: 270, holeD: 25,
		}},
		{"chain ended before each hole", revolvePlaneScene{
			radius: 20, z0: -3.8, z1: 0, bore: 3.25,
			holes: 2, holeR: 0.75, holeAt: 8, holeFirst: 270, holeD: 25, copyEach: true,
		}},
		{"thicker disc, other levels and sizes", revolvePlaneScene{
			radius: 15, z0: 1.3, z1: 6.1, bore: 2.5,
			holes: 2, holeR: 1.1, holeAt: 6.5, holeFirst: 40, holeD: 70, copyEach: true,
		}},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			body, step, err := tc.sc.build(t.Context(), t)
			require.NoError(t, err, "step %s", step)

			_, step, err = tc.sc.build(booleaneval.WithoutRetainedPlaneSlab(t.Context()), t)
			require.Error(t, err, "the control builds every step")
			t.Logf("control refuses at %s", step)
			requireChainDepthRefusal(t, err)

			sc := tc.sc
			h := sc.z1 - sc.z0
			want := math.Pi * (sc.radius*sc.radius - sc.bore*sc.bore) * h
			want -= math.Pi * (sc.groove[1]*sc.groove[1] - sc.groove[0]*sc.groove[0]) * sc.grooveDepth
			want -= float64(sc.holes) * math.Pi * sc.holeR * sc.holeR * h
			got, err := body.Volume()
			require.NoError(t, err)
			// The faceted body chords every circle, so the exact solid lies
			// within the volume's proven bound plus the chord deficit, which
			// the tolerance below covers.
			require.InEpsilon(t, want, got.Value.Mag(), 2e-3)

			checked := 0
			for _, v := range body.Vertices() {
				p := v.Position().Value
				if math.Abs(p.Z-sc.z0) > 1e-9 && math.Abs(p.Z-sc.z1) > 1e-9 {
					continue
				}
				for i := range sc.holes {
					x, y := sc.holeCentre(i)
					off := math.Abs(math.Hypot(p.X-x, p.Y-y) - sc.holeR)
					if off > 0.05 {
						continue
					}
					checked++
					require.LessOrEqual(t, off, v.Position().Bound.Mag()+1e-9, "vertex %v", p)
				}
			}
			require.Positive(t, checked, "the holes leave rim vertices on the planar faces")
		})
	}
}

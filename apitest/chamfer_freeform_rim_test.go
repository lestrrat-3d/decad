package apitest_test

import (
	"context"
	"math"
	"testing"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// This file is agent-docs/modify-reach-design.md §8.5's test obligation: a Chamfer over the whole outer cap
// loop of a prism whose section is a smooth chain of fit splines.

// flower is the polar outline r(θ) = base + amp·cos(lobes·θ).
type flower struct {
	base, amp float64
	lobes     int
}

func (f flower) point(a float64) (float64, float64) {
	r := f.base + f.amp*math.Cos(float64(f.lobes)*a)
	return r * math.Cos(a), r * math.Sin(a)
}

// curvature returns the signed curvature numerator of the outline at angle a.
func (f flower) curvature(a float64) float64 {
	const h = 1e-5
	x0, y0 := f.point(a - h)
	x1, y1 := f.point(a)
	x2, y2 := f.point(a + h)
	dx, dy := (x2-x0)/(2*h), (y2-y0)/(2*h)
	ddx, ddy := (x2-2*x1+x0)/(h*h), (y2-2*y1+y0)/(h*h)
	return dx*ddy - dy*ddx
}

// perimeter returns the outline's length.
func (f flower) perimeter() float64 {
	const steps = 400000
	length := 0.0
	px, py := f.point(0)
	for i := 1; i <= steps; i++ {
		x, y := f.point(2 * math.Pi * float64(i) / steps)
		length += math.Hypot(x-px, y-py)
		px, py = x, y
	}
	return length
}

// splitAngles returns 0, every angle where the curvature changes sign, and 2π.
func (f flower) splitAngles() []float64 {
	cuts := []float64{0}
	const steps = 20000
	prev := f.curvature(1e-4) > 0
	for i := 1; i < steps; i++ {
		a := 2 * math.Pi * float64(i) / steps
		if cur := f.curvature(a) > 0; cur != prev {
			lo, hi := a-2*math.Pi/steps, a
			for range 60 {
				mid := (lo + hi) / 2
				if (f.curvature(mid) > 0) == prev {
					lo = mid
				} else {
					hi = mid
				}
			}
			cuts = append(cuts, (lo+hi)/2)
			prev = cur
		}
	}
	return append(cuts, 2*math.Pi)
}

// stations returns the angles in [lo, hi] at which to place fit points: both ends, and one wherever the
// tangent has turned by turnDeg since the last.
func (f flower) stations(lo, hi, turnDeg float64) []float64 {
	const fine = 4000
	dir := func(a float64) float64 {
		const h = 1e-6
		x0, y0 := f.point(a - h)
		x1, y1 := f.point(a + h)
		return math.Atan2(y1-y0, x1-x0)
	}
	out := []float64{lo}
	last, turn := dir(lo), 0.0
	for i := 1; i < fine; i++ {
		a := lo + (hi-lo)*float64(i)/fine
		d := dir(a)
		delta := d - last
		for delta > math.Pi {
			delta -= 2 * math.Pi
		}
		for delta < -math.Pi {
			delta += 2 * math.Pi
		}
		turn += math.Abs(delta)
		last = d
		if turn >= turnDeg*math.Pi/180 {
			out = append(out, a)
			turn = 0
		}
	}
	return append(out, hi)
}

// draw draws the outline as fit splines cut at its inflections, a point wherever the tangent turns by turnDeg.
func (f flower) draw(t *testing.T, s *sketch.Sketch, turnDeg float64) {
	t.Helper()
	cuts := f.splitAngles()
	var first, shared *sketch.Point
	for i := 0; i+1 < len(cuts); i++ {
		angles := f.stations(cuts[i], cuts[i+1], turnDeg)
		n := len(angles) - 1
		pts := make([]*sketch.Point, n+1)
		for k := range pts {
			switch {
			case k == 0 && shared != nil:
				pts[k] = shared
			case k == n && i+2 == len(cuts):
				pts[k] = first
			default:
				x, y := f.point(angles[k])
				pts[k] = s.CreatePoint(x, y)
				s.Fix(pts[k])
			}
		}
		_, err := s.CreateFitSpline(pts...)
		require.NoError(t, err)
		if first == nil {
			first = pts[0]
		}
		shared = pts[n]
	}
}

// flowerHeight is the sweep of every flower prism, in millimeters.
const flowerHeight = 8

// prism extrudes the flower by flowerHeight.
func (f flower) prism(t *testing.T, doc *decad.Document) *decad.Body {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	f.draw(t, s, 4)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	profiles := s.Profiles()
	require.Len(t, profiles, 1)
	body, err := decad.Extrude(t.Context(), doc, s, profiles[0],
		decad.Distance{D: units.Millimeters(flowerHeight), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func bothRims(body *decad.Body) *decad.EdgeQuery {
	return decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Or(decad.OuterLoopOf(decad.CapEnd(body)))
}

// A 45° chamfer c of a rim with perimeter P removes P·c²/2 − π·c³/3 of volume, because the outline's section
// loses P·s − π·s² of area at inset s. The test chamfers both rims of a twelve-lobed flower and reads the volume
// back through the Boolean's own bound.
func TestChamferFreeformRimVolume(t *testing.T) {
	t.Parallel()
	f := flower{base: 20, amp: 1, lobes: 12}
	doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
	body := f.prism(t, doc)
	sharp, err := body.Volume()
	require.NoError(t, err)

	const c = 0.5
	out, err := decad.Chamfer(t.Context(), body, bothRims(body), units.Millimeters(c))
	require.NoError(t, err)
	warnings := out.OperationDiagnostics()
	require.Len(t, warnings, 1)
	require.Equal(t, decad.OpDiagChainAllowanceApplied, warnings[0].Code)
	require.Equal(t, decad.OperationChamfer, warnings[0].Operation)
	require.Empty(t, warnings[0].Subject.Name)
	require.Nil(t, warnings[0].Subject.Ordinal)
	require.NotNil(t, warnings[0].Chain)
	require.NotNil(t, warnings[0].Chain.ChainAllowance)
	require.Greater(t, warnings[0].Chain.TouchedBound.Base(), warnings[0].Chain.PairChordLimit.Base())
	require.Equal(t, 0.1, warnings[0].Chain.ChainAllowance.Base())
	require.Len(t, doc.Bodies(), 1)
	require.Equal(t, out, doc.Bodies()[0])

	mesh, err := out.Tessellate(t.Context(), units.Millimeters(0.1), decad.WithVerification(decad.VerifyAll))
	require.NoError(t, err)
	require.True(t, mesh.BoundaryVerified())
	require.True(t, mesh.VolumeVerified())
	volume, err := out.Volume()
	require.NoError(t, err)

	want := 2 * (f.perimeter()*c*c/2 - math.Pi*c*c*c/3)
	removed := sharp.Value.Base() - volume.Value.Base()
	require.InDelta(t, want, removed, volume.Bound.Base()+sharp.Bound.Base())
	require.Greater(t, removed, 0.5*want)

	box, err := out.Bounds()
	require.NoError(t, err)
	require.InDelta(t, flowerHeight, box.Max.Z-box.Min.Z, 1e-6, "the chamfer keeps the sweep's height")
}

// Chamfering one rim removes half the wedge of two, and leaves the other rim's edge sharp.
func TestChamferFreeformRimOneCap(t *testing.T) {
	t.Parallel()
	f := flower{base: 20, amp: 1, lobes: 12}
	const c = 0.5
	want := f.perimeter()*c*c/2 - math.Pi*c*c*c/3
	for _, tc := range []struct {
		name string
		rim  func(*decad.Body) *decad.EdgeQuery
	}{
		{"start", func(b *decad.Body) *decad.EdgeQuery { return decad.Edges(decad.OuterLoopOf(decad.CapStart(b))) }},
		{"end", func(b *decad.Body) *decad.EdgeQuery { return decad.Edges(decad.OuterLoopOf(decad.CapEnd(b))) }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
			body := f.prism(t, doc)
			sharp, err := body.Volume()
			require.NoError(t, err)
			out, err := decad.Chamfer(t.Context(), body, tc.rim(body), units.Millimeters(c))
			require.NoError(t, err)
			volume, err := out.Volume()
			require.NoError(t, err)
			require.InDelta(t, want, sharp.Value.Base()-volume.Value.Base(), volume.Bound.Base()+sharp.Bound.Base())
			require.Greater(t, sharp.Value.Base()-volume.Value.Base(), 0.5*want)
			// The chamfered rim's cap is smaller than the far cap by the wedge's footprint.
			caps := map[string]float64{}
			for _, face := range out.Faces() {
				origins := face.Origins()
				if len(origins) == 0 || (origins[0].Role != roleCapStart && origins[0].Role != roleCapEnd) {
					continue
				}
				area, err := face.Area()
				require.NoError(t, err)
				caps[origins[0].Role] = area.Value.Base()
			}
			require.Len(t, caps, 2)
			small, large := caps[roleCapStart], caps[roleCapEnd]
			if tc.name == "end" {
				small, large = large, small
			}
			require.Less(t, small, large-f.perimeter()*c/2, "the chamfered cap lost about perimeter × setback of its area")
		})
	}
}

// A refused rim chamfer leaves the receiver live and the document as it was.
func TestChamferFreeformRimRefusals(t *testing.T) {
	t.Parallel()
	f := flower{base: 20, amp: 1, lobes: 12}
	for _, tc := range []struct {
		name   string
		d      float64
		opts   []decad.ChamferOption
		target error
		text   string
	}{
		{"setback reaches the far end", 4.5, nil, decad.ErrUnsupported, "far end"},
		{"setback past a radius of curvature", 3, nil, decad.ErrDegenerate, "radius of curvature"},
	} {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
			body := f.prism(t, doc)
			_, err := decad.Chamfer(t.Context(), body, bothRims(body), units.Millimeters(tc.d), tc.opts...)
			require.ErrorIs(t, err, tc.target)
			require.ErrorContains(t, err, tc.text)
			require.Equal(t, []*decad.Body{body}, doc.Bodies())
		})
	}

	t.Run("a partial loop", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		body := f.prism(t, doc)
		edges, err := decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).SelectEdges(body)
		require.NoError(t, err)
		require.Greater(t, len(edges), 1)
		_, err = decad.Chamfer(t.Context(), body, decad.Edges(decad.OuterLoopOf(decad.CapStart(body))).Exactly(1), units.Millimeters(0.5))
		require.Error(t, err)
		require.Equal(t, []*decad.Body{body}, doc.Bodies())
	})

	t.Run("a canceled context", func(t *testing.T) {
		t.Parallel()
		doc := decad.New(decad.WithChainTolerance(units.Millimeters(0.1)))
		body := f.prism(t, doc)
		ctx, cancel := context.WithCancel(t.Context())
		cancel()
		_, err := decad.Chamfer(ctx, body, bothRims(body), units.Millimeters(0.5))
		require.ErrorIs(t, err, context.Canceled)
		require.Equal(t, []*decad.Body{body}, doc.Bodies())
	})
}

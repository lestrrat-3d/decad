package decad

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bandUnderTest is one chamfered circular cap loop read back as the survey
// reads it: the payload's own patch geometry, the Face DX7 samples, and the
// plane-local map both go through.
type bandUnderTest struct {
	face *Face
	pl   prismPayload
	geom capPatchGeom
}

// chamferedCircularBand chamfers one extruded section's end cap and returns
// its single CIRCULAR band patch. section draws the sketch the extrusion runs
// on, so one caller can hand it a whole disk (a cornerless loop, whose band is
// one whole turn) and another a quarter disk (a wall mitered at both ends,
// whose band patch reads a genuine window).
func chamferedCircularBand(t *testing.T, section func(*sketch.Sketch), h, d float64, motion *r3.Transform) bandUnderTest {
	t.Helper()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	section(s)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	require.Len(t, s.Profiles(), 1)

	doc := New()
	body, err := doc.Extrude(s, s.Profiles()[0], Distance{D: units.Millimeters(h), Dir: Along})
	require.NoError(t, err)
	chamfered, err := body.Chamfer(Edges(CreatedBy(CapEnd(body))), units.Millimeters(d))
	require.NoError(t, err)
	if motion != nil {
		chamfered, err = chamfered.Placed(*motion)
		require.NoError(t, err)
	}

	cbp, ok := chamfered.payload.(capBlendPayload)
	require.True(t, ok)
	roles := facesByRole(chamfered)
	var out bandUnderTest
	found := 0
	for _, patch := range cbp.patches {
		if !patch.geom.circular {
			continue
		}
		found++
		face := roles[patch.role]
		require.NotNil(t, face)
		require.Equal(t, KindCone, face.Surface().Kind())
		// The sweep levels the payload itself carries: the reading under test
		// takes its own levels from the patch geometry, so these only pin that
		// the survey's plane-local map is the payload's own.
		out = bandUnderTest{face: face, pl: cbp.prismLike(cbp.z0, cbp.z1), geom: patch.geom}
	}
	require.Equal(t, 1, found, "these sections each carry one circular wall")
	return out
}

func diskSection(cx, cy, r float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		c := s.CreatePoint(cx, cy)
		s.CreateCircle(c, r)
		s.Fix(c)
	}
}

// quarterDiskSection is the same shape quarterDiskBody builds for the external
// survey tests: one circular wall meeting a straight neighbour at a
// non-tangential miter at BOTH ends, so its band patch reads a quarter-turn
// window rather than a whole period.
//
// Unlike diskSection, this one is drawn at the sketch's own origin and carried
// out into the world by its PLACEMENT. The arc has to weld to a line at each
// end, and sketch judges such a weld against the section's own extent
// (geom.weldIdentEps times the source's extent), so a half-millimetre section
// drawn a metre out leaves that weld a handful of ulps of the coordinates it
// is welding — a margin each platform's own arithmetic can land either side
// of. A closed circle welds to nothing and so has no such corner.
func quarterDiskSection(cx, cy, r float64) func(*sketch.Sketch) {
	return func(s *sketch.Sketch) {
		o := s.CreatePoint(cx, cy)
		s.Fix(o)
		px := s.CreatePoint(cx+r, cy)
		py := s.CreatePoint(cx, cy+r)
		s.CreateLine(o, px)
		s.CreateLine(py, o)
		s.CreateArc(o, px, py)
	}
}

// componentAt reads the band's own published normal component against the pull
// at one azimuth of its window, beside the bound that reading publishes. It is
// the survey's own sampling arm, used here as an INDEPENDENT witness: the
// component of the patch's exact normal at that point lies within the returned
// bound of the returned value, so a range that excludes the whole bracket is
// excluding a value the patch really takes.
func (b bandUnderTest) componentAt(t *testing.T, theta float64, pull r3.Vec) (float64, float64) {
	t.Helper()
	sin, cos := math.Sincos(theta)
	g := b.geom
	n, err := b.face.NormalAt(b.pl.point(g.cU+g.capRadius*cos, g.cV+g.capRadius*sin, g.capZ))
	require.NoError(t, err)
	bound, err := n.Bound.In(units.One)
	require.NoError(t, err)
	return n.Value.Dot(pull), bound * pull.Len()
}

// TestCapPatchNormalRangeCoversWhatThePatchTakes is the sampled half of the
// DX7 reading's proof. The survey recovers its A·cos+B·sin+C form from three
// readings taken at points it computes itself — a float sine and cosine of an
// azimuth, then two rounded maps into world space — and the azimuth those
// points really carry is not the one the recovery reads them as. That
// displacement scales like the frame origin's own rounding divided by the
// patch's radius, so a small band whose placement carries it far from the
// world origin makes it hundreds of times what any reading's own bound covers.
// A case that means to charge that displacement says how far out its patch has
// to sit, and the run checks that its centre really got there.
//
// Whatever it is, the range the survey reports must still hold every value the
// patch takes: each independently read component, widened by its own published
// bound, must sit inside [lo-allow, hi+allow].
func TestCapPatchNormalRangeCoversWhatThePatchTakes(t *testing.T) {
	spin, err := r3.Rotation(r3.NewVec(1, 2, 3), units.Degrees(37))
	require.NoError(t, err)
	shift, err := r3.Translation(r3.NewVec(1000, 1000, 0))
	require.NoError(t, err)
	// The same motion a metre out: what the reading pays for is the rounding of
	// the placed frame's own origin, so a section drawn at the sketch origin and
	// carried out here is displaced exactly as one drawn out there would be.
	spun, err := spin.Then(shift)
	require.NoError(t, err)
	pull, ok := r3.NewVec(1, 0.3, 0.2).Normalize()
	require.True(t, ok)

	for _, tc := range []struct {
		name string
		// overArmBounds is how many times the three sampled readings' OWN
		// published bounds the allowance must exceed. Those bounds are all a
		// reading that charged the arm alone ever had, and they say nothing about
		// where the point handed to the arm sits.
		overArmBounds  float64
		underArmBounds float64
		// centreFar is how far from the world origin the patch's own centre must
		// end up. A case charging the frame origin's rounding has to be placed
		// out there for the charge to mean anything.
		centreFar float64
		section   func(*sketch.Sketch)
		motion    *r3.Transform
	}{
		{
			// An axis-aligned band about the origin: the sample points land where
			// they were asked to, so there is nothing beyond the readings' own
			// arithmetic to pay for.
			name: `whole turn at the origin`, section: diskSection(0, 0, 12),
			underArmBounds: 10,
		},
		{
			// ulp(1000) over a radius of half a millimetre displaces each sample
			// by some 2e-13 of azimuth, which is two orders more than the
			// readings' own bounds cover.
			name: `whole turn far from the origin, placed`, section: diskSection(1000, 1000, 0.55), motion: &spin,
			overArmBounds: 50, centreFar: 1000,
		},
		{
			// A genuine window rather than a whole period, so the reported ends are
			// the window's own rather than the form's amplitude. No ratio is
			// claimed here: this patch is mitered, and the surface-departure bound
			// its readings carry dwarfs every arithmetic term on a band this small.
			// The section is drawn at the sketch origin and placed out here
			// (quarterDiskSection), which displaces the sample points exactly as
			// drawing it out here would.
			name: `quarter window far from the origin, placed`, section: quarterDiskSection(0, 0, 0.55), motion: &spun,
			centreFar: 1000,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			band := chamferedCircularBand(t, tc.section, 10, 0.2, tc.motion)
			lo, hi, allow, ok := capPatchNormalRange(band.face, band.pl, band.geom, pull)
			require.True(t, ok)

			g := band.geom
			if tc.centreFar > 0 {
				require.Greater(t, band.pl.point(g.cU, g.cV, g.capZ).Len(), tc.centreFar,
					"this case charges the frame origin's rounding, so its patch must sit that far out")
			}
			arms := 0.0
			for _, off := range []float64{0, math.Pi / 2, math.Pi} {
				_, bound := band.componentAt(t, g.th0+off, pull)
				arms += bound
			}
			require.Positive(t, arms, "a Cone arm's own cosine and sine are not exact")
			if tc.overArmBounds > 0 {
				require.Greater(t, allow, tc.overArmBounds*arms,
					"the displaced sample points cost more than the readings' own bounds")
			}
			if tc.underArmBounds > 0 {
				require.Less(t, allow, tc.underArmBounds*arms,
					"an undisplaced band must not be charged for a displacement it has not got")
			}
			const samples = 512
			low, high := math.Inf(1), math.Inf(-1)
			for k := range samples + 1 {
				theta := g.th0 + (g.th1-g.th0)*float64(k)/samples
				v, bound := band.componentAt(t, theta, pull)
				require.GreaterOrEqual(t, v+bound, lo-allow,
					"azimuth %v takes a component below the reported range", theta)
				require.LessOrEqual(t, v-bound, hi+allow,
					"azimuth %v takes a component above the reported range", theta)
				low, high = math.Min(low, v), math.Max(high, v)
			}
			// A range wide enough to hold anything would pass the containment
			// above for free. These witnesses come within the grid's own reach of
			// both ends, so the reported ends are the patch's own.
			require.Less(t, high-hi, 1e-4, "the reported maximum runs past what the patch reaches")
			require.Less(t, lo-low, 1e-4, "the reported minimum runs past what the patch reaches")
		})
	}
}

// TestHarmonicWindowRangeEnclosesInteriorExtremes is the reading half of the
// same proof. A window whose interior holds the form's stationary point takes
// its whole amplitude there, and a float64 evaluation of that extreme — a sine
// and a cosine at each end, an arctangent for the stationary point, a
// multiply-add at each candidate — can report an interval the exact extreme
// lies OUTSIDE of. The enclosure must bracket c±R, and must place its own
// reported ends outside every value the window actually takes.
func TestHarmonicWindowRangeEnclosesInteriorExtremes(t *testing.T) {
	for _, tc := range []struct {
		name       string
		a, b, c    *big.Rat
		width      *big.Rat
		wholeTurn  bool
		interior   bool
		tightBelow float64
	}{
		{
			name: `stationary point inside`, a: big.NewRat(3, 7), b: big.NewRat(-5, 11), c: big.NewRat(1, 3),
			width: big.NewRat(11, 2), interior: true, tightBelow: 1e-15,
		},
		{
			name: `endpoints only`, a: big.NewRat(1, 1), b: big.NewRat(0, 1), c: big.NewRat(0, 1),
			width: big.NewRat(1, 1), interior: false, tightBelow: 1e-15,
		},
		{
			name: `whole turn`, a: big.NewRat(2, 5), b: big.NewRat(9, 8), c: big.NewRat(-1, 4),
			width: big.NewRat(0, 1), wholeTurn: true, interior: true, tightBelow: 1e-15,
		},
	} {
		t.Run(tc.name, func(t *testing.T) {
			ext, ok := harmonicWindowRange(tc.a, tc.b, tc.c, tc.width, tc.wholeTurn)
			require.True(t, ok)
			require.LessOrEqual(t, ext.minLo.Cmp(ext.minHi), 0)
			require.LessOrEqual(t, ext.maxLo.Cmp(ext.maxHi), 0)
			require.Less(t, ratFloatUp(new(big.Rat).Sub(ext.minHi, ext.minLo)), tc.tightBelow)
			require.Less(t, ratFloatUp(new(big.Rat).Sub(ext.maxHi, ext.maxLo)), tc.tightBelow)

			amp, okAmp := intervalSqrt(pointInterval(ratAdd(ratMul(tc.a, tc.a), ratMul(tc.b, tc.b))))
			require.True(t, okAmp)
			if tc.interior {
				// The stationary points are reached, so both extremes are the
				// form's own amplitude about c and the enclosure must bracket them.
				trough := intervalSub(pointInterval(tc.c), amp)
				peak := intervalAdd(pointInterval(tc.c), amp)
				require.LessOrEqual(t, ext.minLo.Cmp(trough.hi), 0)
				require.GreaterOrEqual(t, ext.minHi.Cmp(trough.lo), 0)
				require.GreaterOrEqual(t, ext.maxHi.Cmp(peak.lo), 0)
				require.LessOrEqual(t, ext.maxLo.Cmp(peak.hi), 0)
			}

			// No azimuth of the window may take a value the reported enclosure
			// says is out of reach — including the stationary point itself,
			// where the float reading's own rounding used to escape.
			width := tc.width
			if tc.wholeTurn {
				width = ratMul(big.NewRat(2, 1), piUpper)
			}
			stationary, okStat := ratOf(math.Atan2(ratFloat(tc.b), ratFloat(tc.a)))
			require.True(t, okStat)
			for k := range 129 {
				phi := ratMul(width, big.NewRat(int64(k), 128))
				for _, at := range []*big.Rat{phi, new(big.Rat).Add(stationary, phi), new(big.Rat).Sub(stationary, phi)} {
					if at.Cmp(new(big.Rat)) < 0 || at.Cmp(width) > 0 {
						continue
					}
					sin, cos, okT := radSinCosInterval(at)
					require.True(t, okT)
					v := intervalAdd(intervalAdd(intervalScale(cos, tc.a), intervalScale(sin, tc.b)), pointInterval(tc.c))
					require.LessOrEqual(t, ext.minLo.Cmp(v.hi), 0, "a reachable value sits below the reported minimum")
					require.GreaterOrEqual(t, ext.maxHi.Cmp(v.lo), 0, "a reachable value sits above the reported maximum")
				}
			}
		})
	}
}

func ratFloat(q *big.Rat) float64 {
	f, _ := q.Float64()
	return f
}

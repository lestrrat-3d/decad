package apitest_test

import (
	"fmt"
	"math"
	"testing"
	"time"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// bandPlacement is one rigid placement of the pin-in-bore pair: a rotation
// about axis through (5, 5, 5), by degrees.
type bandPlacement struct {
	name string
	axis r3.Vec
	deg  float64
}

func bandPlacements() []bandPlacement {
	out := []bandPlacement{
		{"x0", r3.NewVec(0, 1, 0), 0},
	}
	for _, d := range []float64{5, 20, 45, 90, 137} {
		out = append(out, bandPlacement{fmt.Sprintf("y%v", d), r3.NewVec(0, 1, 0), d})
	}
	return append(out,
		bandPlacement{"x30", r3.NewVec(1, 0, 0), 30},
		bandPlacement{"xy30", r3.NewVec(1, 1, 0), 30},
	)
}

// borePlate builds a 20x20x10 plate with a bore of diameter 6.4 at (10, 10),
// made by Cut when cut is true and by extruding a holed profile otherwise.
func borePlate(t *testing.T, doc *decad.Document, cut bool) *decad.Body {
	t.Helper()
	if cut {
		plate := boxBodyAtZ(t, doc, 0, 0, 20, 20, 0, 10)
		tool := circlePrismAtZ(t, doc, 10, 10, 3.2, -1, 12)
		body, err := decad.Cut(t.Context(), plate, tool)
		require.NoError(t, err)
		return body
	}
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	require.NoError(t, err)
	rect := s.CreateRectangle(0, 0, 20, 20)
	s.Fix(rect.A)
	centre := s.CreatePoint(10, 10)
	s.Fix(centre)
	s.CreateCircle(centre, 3.2)
	_, err = s.Solve(t.Context())
	require.NoError(t, err)
	var holed *sketch.Profile
	for _, p := range s.Profiles() {
		if len(p.Holes) > 0 {
			holed = p
		}
	}
	require.NotNil(t, holed)
	body, err := decad.Extrude(t.Context(), doc, s, holed, decad.Distance{D: units.Millimeters(10), Dir: decad.Along})
	require.NoError(t, err)
	return body
}

func placement(t *testing.T, p bandPlacement, fraction float64) r3.Transform {
	t.Helper()
	rot, err := r3.RotationAround(r3.NewVec(5, 5, 5), p.axis, units.Degrees(p.deg*fraction))
	require.NoError(t, err)
	return rot
}

// placedPinInBore builds the plate and a pin of diameter pinD through it,
// placed by p: the plate by one rotation, the pin by one rotation, or by two
// half rotations when split.
func placedPinInBore(t *testing.T, p bandPlacement, cut, split bool, pinD float64) (*decad.Document, *decad.Body, *decad.Body) {
	t.Helper()
	doc := decad.New()
	plate := borePlate(t, doc, cut)
	pin := circlePrismAtZ(t, doc, 10, 10, pinD/2, -2, 17)
	if p.deg == 0 {
		return doc, plate, pin
	}
	plate, err := decad.Transform(t.Context(), plate, placement(t, p, 1))
	require.NoError(t, err)
	turns := []float64{1}
	if split {
		turns = []float64{0.5, 0.5}
	}
	for _, f := range turns {
		pin, err = decad.Transform(t.Context(), pin, placement(t, p, f))
		require.NoError(t, err)
	}
	return doc, plate, pin
}

// TestClearancePlacedPinInBore pins that a pin of diameter 6 through a bore of
// diameter 6.4, both placed by one rigid rotation, measures its 0.2 mm
// clearance. The rotation lands the bore's and the pin's rim centres a few
// ulps off a common axis, so the degeneracy oracle answers neither coaxial nor
// not; the rim, plane-wall and rim-height bands of
// agent-docs/clearance-band-design.md §4 certify the pair. Seen red without
// them: the 20, 45, 137 degree and off-Y rows read Suspect with
// DiagUndecidedClearance.
func TestClearancePlacedPinInBore(t *testing.T) {
	t.Parallel()
	for _, cut := range []bool{true, false} {
		for _, p := range bandPlacements() {
			name := fmt.Sprintf("%s/cut=%v", p.name, cut)
			t.Run(name, func(t *testing.T) {
				t.Parallel()
				doc, plate, pin := placedPinInBore(t, p, cut, false, 6)
				report, err := doc.Verify(t.Context(), decad.WithClearances())
				require.NoError(t, err)
				require.Equal(t, decad.Sound, report.Status, "%v", report.Diagnostics)
				require.Len(t, report.Clearances, 1)
				row := report.Clearances[0]
				require.ElementsMatch(t, []*decad.Body{plate, pin}, []*decad.Body{row.A, row.B})
				require.InDelta(t, 0.2, row.Gap.Value.Mag(), 1e-9)
			})
		}
	}
}

// TestClearancePlacedPinTwoPlacements pins the pair whose plate is placed by
// one rotation and whose pin is placed by two half rotations, so the two axes'
// float directions differ in their last bits. The pin's end caps and the
// plate's faces then stand a few roundings off parallel, and a plane face over
// a plane is the flat-feature reading of
// agent-docs/clearance-band-design.md §5. Until that reading lands, a placement
// whose boxes meet there stays undecided: Suspect with
// DiagUndecidedClearance and no row. Every other placement reads the single
// placement's row, Sound, one clearance, 0.2 mm. No placement reads a wrong
// gap.
func TestClearancePlacedPinTwoPlacements(t *testing.T) {
	t.Parallel()
	undecided := map[string]struct{}{"y45": {}, "y137": {}}
	for _, p := range bandPlacements() {
		t.Run(p.name, func(t *testing.T) {
			t.Parallel()
			doc, _, _ := placedPinInBore(t, p, true, true, 6)
			report, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			if _, ok := undecided[p.name]; ok {
				require.Empty(t, report.Clearances)
				require.Equal(t, decad.Suspect, report.Status)
				_, found := findDiagnostic(report.Diagnostics, decad.DiagUndecidedClearance)
				require.True(t, found)
				return
			}
			require.Equal(t, decad.Sound, report.Status, "%v", report.Diagnostics)
			require.Len(t, report.Clearances, 1)
			require.InDelta(t, 0.2, report.Clearances[0].Gap.Value.Mag(), 1e-9)
		})
	}
}

// TestClearancePlacedPinThatFillsItsBore pins that a pin whose radius equals
// its bore's reads no band: no clearance row, and the pair stays Suspect.
func TestClearancePlacedPinThatFillsItsBore(t *testing.T) {
	t.Parallel()
	p := bandPlacement{"y20", r3.NewVec(0, 1, 0), 20}
	doc, _, _ := placedPinInBore(t, p, true, false, 6.4)
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Empty(t, report.Clearances)
	require.Equal(t, decad.Suspect, report.Status)
}

// TestClearancePlacedPinDifferential checks every certified row against a
// brute-force distance over the tessellation VERTICES of the two bodies, which
// lie on the boundary: the proven lower bound never exceeds the sampled
// minimum, and the upper bound plus the covering radius reaches it. A facet's
// interior lies inside a concave bore, so facet points would understate the
// minimum.
func TestClearancePlacedPinDifferential(t *testing.T) {
	t.Parallel()
	placements := []bandPlacement{
		{"y20", r3.NewVec(0, 1, 0), 20},
		{"y137", r3.NewVec(0, 1, 0), 137},
		{"xy30", r3.NewVec(1, 1, 0), 30},
	}
	if raceDetector {
		placements = placements[:1]
	}
	for _, p := range placements {
		t.Run(p.name, func(t *testing.T) {
			t.Parallel()
			doc, plate, pin := placedPinInBore(t, p, true, false, 6)
			report, err := doc.Verify(t.Context(), decad.WithClearances())
			require.NoError(t, err)
			require.Len(t, report.Clearances, 1)
			vertices := func(b *decad.Body) ([]r3.Vec, float64) {
				mesh, err := b.Tessellate(t.Context(), units.Millimeters(0.01))
				require.NoError(t, err)
				maxEdge := 0.0
				for _, tri := range mesh.Triangles() {
					v := mesh.Vertices()
					for i := range 3 {
						maxEdge = math.Max(maxEdge, v[tri[i]].Sub(v[tri[(i+1)%3]]).Len())
					}
				}
				return mesh.Vertices(), maxEdge
			}
			pa, ea := vertices(plate)
			pb, eb := vertices(pin)
			assertBracket(t, report.Clearances[0], minPairDistance(pa, pb), ea+eb, "placed pin in bore")
		})
	}
}

// TestClearancePlacedPinInBoreIsFast pins the cost of the repro: a placed pin
// in its bore verifies in milliseconds, not seconds.
func TestClearancePlacedPinInBoreIsFast(t *testing.T) {
	t.Parallel()
	p := bandPlacement{"y20", r3.NewVec(0, 1, 0), 20}
	doc, _, _ := placedPinInBore(t, p, true, false, 6)
	start := time.Now()
	report, err := doc.Verify(t.Context(), decad.WithClearances())
	require.NoError(t, err)
	require.Len(t, report.Clearances, 1)
	require.Less(t, time.Since(start), 5*time.Second)
}

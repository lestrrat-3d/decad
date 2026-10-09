package decad

import (
	"math/big"
	"testing"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
	"github.com/stretchr/testify/require"
)

// TestOffsetSectionDeltaEnclosesDenotedOffset shells four plates with a
// 0.1 in wall through the real Shell and checks the offset section each cup
// records against the offset its denoted thickness names. The denoted
// thickness t* is 0.1 in rescaled to millimetres in exact rationals, and every
// plate's walls are axis-aligned, so each denoted corner — a miter, an arc
// foot, a G1 point — and each denoted circle radius is an exact rational the
// test spells directly. Every recorded segment endpoint must sit within one
// third of the cup's offsetDelta of the denoted point it stands for (the
// per-point reach offsetSectionDelta triples), and every recorded circle
// radius within that of its denoted radius.
//
// Legs shown to fail (each deleted in offset2d.LoopReach, the fixture watched go
// red, then restored):
//   - The miter enclosure: with its reach dropped, the inward rectangle and
//     the holed plate miss a cavity corner.
//   - The arc-corner feet: with their reach dropped, the outward rectangle
//     misses a foot.
//   - The G1 hull: with its reach dropped, the slot misses a flank end.
//   - The concentric circle's radial gap: with it dropped, the disk's cavity
//     radius misses.
//   - The thickness interval: with amount taken at the held thickness alone,
//     the outward rectangle misses a foot whose only error is the conversion.
//
// Two legs cannot be exhibited here. A recorded arc whose end sits off the
// circle its start fixes (circularWalkEndGap) needs an inconsistent arc
// record, and no sketch solve this test can drive produces one at a
// measurable distance. The factor three covers points INSIDE a recorded arc,
// which a vertex check does not reach; offsetSectionDelta's doc comment
// carries its argument.
func TestOffsetSectionDeltaEnclosesDenotedOffset(t *testing.T) {
	thickness := units.Inches(0.1)
	wall := new(big.Rat).Quo(
		new(big.Rat).Mul(proofarith.FloatRat(thickness.Mag()), proofarith.FloatRat(thickness.Unit().Factor())),
		proofarith.FloatRat(units.Millimeter.Factor()))
	r := func(v float64) *big.Rat { return proofarith.FloatRat(v) }
	add := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Add(a, b) }
	sub := func(a, b *big.Rat) *big.Rat { return new(big.Rat).Sub(a, b) }
	neg := func(a *big.Rat) *big.Rat { return new(big.Rat).Neg(a) }
	rectangle := func(s *sketch.Sketch) {
		rect := s.CreateRectangle(0, 0, 100, 60)
		s.Fix(rect.A)
	}
	// The inward rectangle's four miters.
	miters := [][2]*big.Rat{
		{wall, wall}, {sub(r(100), wall), wall},
		{sub(r(100), wall), sub(r(60), wall)}, {wall, sub(r(60), wall)},
	}
	tests := []struct {
		name   string
		draw   func(*sketch.Sketch)
		sense  ShellSense
		points [][2]*big.Rat
		radii  []*big.Rat
	}{
		{name: "inward rectangle", draw: rectangle, sense: Inward, points: miters},
		{
			name: "outward rectangle", draw: rectangle, sense: Outward,
			points: [][2]*big.Rat{
				{r(0), neg(wall)}, {r(100), neg(wall)}, {add(r(100), wall), r(0)}, {add(r(100), wall), r(60)},
				{r(100), add(r(60), wall)}, {r(0), add(r(60), wall)}, {neg(wall), r(60)}, {neg(wall), r(0)},
			},
		},
		{
			name: "holed plate",
			draw: func(s *sketch.Sketch) {
				rectangle(s)
				s.CreateCircle(s.CreatePoint(50, 30), 8)
			},
			sense: Inward, points: miters, radii: []*big.Rat{add(r(8), wall)},
		},
		{
			name: "disk",
			draw: func(s *sketch.Sketch) {
				center := s.CreatePoint(0, 0)
				s.Fix(center)
				s.CreateCircle(center, 20)
			},
			sense: Inward, radii: []*big.Rat{sub(r(20), wall)},
		},
		{
			name: "slot",
			draw: func(s *sketch.Sketch) {
				slot, err := s.CreateSlot(36, -22, 36, 22, 12)
				require.NoError(t, err)
				s.Fix(slot.C1)
				s.Fix(slot.C2)
			},
			sense: Inward,
			points: [][2]*big.Rat{
				{add(r(24), wall), r(-22)}, {add(r(24), wall), r(22)},
				{sub(r(48), wall), r(-22)}, {sub(r(48), wall), r(22)},
			},
		},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			w := sketch.NewWorld()
			s, err := w.CreateSketch(w.XY())
			require.NoError(t, err)
			tt.draw(s)
			_, err = s.Solve(t.Context())
			require.NoError(t, err)
			profile := s.Profiles()[0]
			for _, p := range s.Profiles() {
				if len(p.Holes) > len(profile.Holes) {
					profile = p
				}
			}
			box, err := New().Extrude(s, profile, Distance{D: units.Millimeters(10), Dir: Along})
			require.NoError(t, err)
			cup, err := box.Shell(t.Context(), Faces(FaceCreatedBy(CapEnd(box))), thickness, WithShellSense(tt.sense))
			require.NoError(t, err)
			record, ok := cup.payload.(cupPayload)
			require.True(t, ok)
			cp := record.view()
			// offsetDelta is at least three times the reach, exactly.
			reach := new(big.Rat).Quo(proofarith.FloatRat(cp.offsetDelta), big.NewRat(3, 1))
			reach2 := new(big.Rat).Mul(reach, reach)

			offset := cp.cavity
			if tt.sense == Outward {
				offset = cp.outer
			}
			var radii []*big.Rat
			for _, loop := range append([]loopRecord{offset.Outer}, offset.Holes...) {
				for _, seg := range loop.Segments {
					var ends []Point2
					switch g := seg.(type) {
					case lineSeg:
						ends = []Point2{g.Start, g.End}
					case arcSeg:
						ends = []Point2{g.Start, g.End}
					case circleSeg:
						rr, err := g.Radius.In(units.Millimeter)
						require.NoError(t, err)
						radii = append(radii, r(rr))
						continue
					default:
						t.Fatalf("unexpected offset segment %T", seg)
					}
					for _, p := range ends {
						requireNearDenoted(t, p, tt.points, reach2)
					}
				}
			}
			require.Len(t, radii, len(tt.radii))
			for i, held := range radii {
				gap := new(big.Rat).Abs(sub(held, tt.radii[i]))
				require.LessOrEqual(t, gap.Cmp(reach), 0, "circle %d radius misses its denoted radius", i)
			}
			// Every fixture's record differs from its denotation, so a zero
			// displacement would be a claim the checks above only failed to
			// catch.
			require.Positive(t, cp.offsetDelta)
		})
	}
}

// requireNearDenoted checks that p lies within sqrt(reach2) of one of the
// denoted points, compared in exact rationals.
func requireNearDenoted(t *testing.T, p Point2, denoted [][2]*big.Rat, reach2 *big.Rat) {
	t.Helper()
	pu, pv := proofarith.FloatRat(p.U), proofarith.FloatRat(p.V)
	var best *big.Rat
	for _, q := range denoted {
		du, dv := new(big.Rat).Sub(pu, q[0]), new(big.Rat).Sub(pv, q[1])
		d2 := new(big.Rat).Add(new(big.Rat).Mul(du, du), new(big.Rat).Mul(dv, dv))
		if best == nil || d2.Cmp(best) < 0 {
			best = d2
		}
	}
	require.LessOrEqual(t, best.Cmp(reach2), 0, "recorded point (%v, %v) misses every denoted point", p.U, p.V)
}

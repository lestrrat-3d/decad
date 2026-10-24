package revolveshell_test

import (
	"math"
	"math/big"
	"testing"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/massmoment"
	"github.com/lestrrat-3d/decad/internal/pointcone"
	"github.com/lestrrat-3d/decad/internal/revolveshell"
	"github.com/lestrrat-3d/r3"
	"github.com/stretchr/testify/require"
)

const conePrec = 512

// sectorContact is the first root contact of the test sector. Every node
// coordinate stays below 8 in magnitude, so the placement rounding allowance
// is the one for coordinates under 8 and the drift dominates it.
var sectorContact = r3.Vec{X: 6.5, Y: 6.5, Z: 6.5}

func bigOf(v float64) *big.Float { return new(big.Float).SetPrec(conePrec).SetFloat64(v) }

func bigDot(a, b [3]*big.Float) *big.Float {
	out := new(big.Float).SetPrec(conePrec)
	for i := range a {
		out.Add(out, new(big.Float).SetPrec(conePrec).Mul(a[i], b[i]))
	}
	return out
}

func bigVec(v r3.Vec) [3]*big.Float { return [3]*big.Float{bigOf(v.X), bigOf(v.Y), bigOf(v.Z)} }

// coneReading returns the axial h = x·Â and radial ρ = |x − hÂ| of x about the
// exact unit axis Â along axis, in 512-bit arithmetic.
func coneReading(x, axis r3.Vec) (*big.Float, *big.Float) {
	a := bigVec(axis)
	norm := new(big.Float).SetPrec(conePrec).Sqrt(bigDot(a, a))
	p := bigVec(x)
	h := new(big.Float).SetPrec(conePrec).Quo(bigDot(p, a), norm)
	rhoSq := new(big.Float).SetPrec(conePrec).Sub(bigDot(p, p), new(big.Float).SetPrec(conePrec).Mul(h, h))
	if rhoSq.Sign() < 0 {
		rhoSq.SetInt64(0)
	}
	return h, new(big.Float).SetPrec(conePrec).Sqrt(rhoSq)
}

// coneDistance is the distance from x to the nappe with apex at the origin,
// about axis, that passes through generator: |ρ·h₀ − h·ρ₀|/|g| when x projects
// onto the generator's ray, and |x| otherwise.
func coneDistance(x, generator, axis r3.Vec) *big.Float {
	h, rho := coneReading(x, axis)
	h0, rho0 := coneReading(generator, axis)
	g := new(big.Float).SetPrec(conePrec).Sqrt(bigDot(bigVec(generator), bigVec(generator)))
	along := new(big.Float).SetPrec(conePrec).Add(new(big.Float).SetPrec(conePrec).Mul(rho, rho0),
		new(big.Float).SetPrec(conePrec).Mul(h, h0))
	if along.Sign() < 0 {
		return new(big.Float).SetPrec(conePrec).Sqrt(bigDot(bigVec(x), bigVec(x)))
	}
	d := new(big.Float).SetPrec(conePrec).Sub(new(big.Float).SetPrec(conePrec).Mul(rho, h0),
		new(big.Float).SetPrec(conePrec).Mul(h, rho0))
	d.Abs(d)
	return d.Quo(d, g)
}

// sectorPlanInput is a sector cap of two root contacts and one more node,
// placed at identity and at motion about axis, with every bound small enough
// that only the tested charge decides.
func sectorPlanInput(t *testing.T, axis r3.Vec, motion r3.Transform) revolveshell.SectorJoinInput {
	t.Helper()
	contact := sectorContact
	nodes := []pointcone.SectorNode{
		{P: contact, Crossing: 0, RootBoundary: true},
		{P: turnAbout(t, axis, 0.1).Apply(contact), Crossing: 1, RootBoundary: true},
		{P: contact.Scale(0.5), Crossing: -1},
	}
	edges := []pointcone.SectorEdge{{U: 1, V: 2}, {U: 2, V: 0}, {U: 0, V: 1, Group: -1}}
	motions := []r3.Transform{r3.Identity(), motion}
	charges := make([]massmoment.MapCharge, len(motions))
	drifts := make([]float64, len(motions))
	for i, m := range motions {
		var err error
		charges[i], drifts[i], err = massmoment.AxisPlacementCharge(m, axisValue(t, axis))
		require.NoError(t, err)
	}
	seam, err := facetproof.SectorSeam(nodes[0].P, nodes[1].P, motions)
	require.NoError(t, err)
	return revolveshell.SectorJoinInput{
		Original: nodes, RootContactIndex: 1, Edges: edges,
		Motions: motions, Charges: charges, Drifts: drifts,
		SourceMeshBounds: []float64{0.001, 0.001}, SourceCrossingBound: 0.001,
		Seam: seam, MinRadius: 1, RegularSource: make([]float64, 18),
	}
}

// A copy whose axis drifts by just under DriftSinLimit moves its root contacts
// off the root cone by that drift times their radius. PlacedRound must cover
// it: the exact rotation Q about the axis maps the cone onto itself, so a
// placed contact can be no farther from the cone than its source contact plus
// |Mp − Qp| and the rounding of Mp.
func TestPrepareSectorJoinPlansChargesDriftAboutAxis(t *testing.T) {
	t.Parallel()
	for name, axis := range generalAxes {
		t.Run(name, func(t *testing.T) {
			t.Parallel()
			contact := sectorContact
			motion := tiltedTurnAbout(t, axis, math.Pi, 0x1.8p-47, contact)
			input := sectorPlanInput(t, axis, motion)
			require.Greater(t, input.Drifts[1], input.Charges[1].Stretch,
				"the drift charge exceeds the bare Stretch")
			plans, _, err := revolveshell.PrepareSectorJoinPlans(input)
			require.NoError(t, err)
			require.Len(t, plans, 2)
			for j, node := range input.Original {
				if !node.RootBoundary {
					continue
				}
				placed := plans[1].Nodes[j].P
				moved := new(big.Float).SetPrec(conePrec).Sub(coneDistance(placed, contact, axis),
					coneDistance(node.P, contact, axis))
				require.LessOrEqual(t, moved.Cmp(bigOf(plans[1].PlacedRound)), 0,
					"root contact %d moved %s off the cone, PlacedRound %g", j, moved.Text('g', 6),
					plans[1].PlacedRound)
			}
		})
	}
}

// The seam cost of a general plan carries τ = (π/2)·δ_B·r_max. A frame tilt
// that makes 2τ/minRadius exceed the copies' angular gap refuses them as not
// separate, and a quarter of it does not.
func TestPrepareSectorJoinPlansChargesFrameTilt(t *testing.T) {
	t.Parallel()
	axis := r3.Vec{X: 1, Y: 2}
	input := sectorPlanInput(t, axis, turnAbout(t, axis, 1.2))
	plans, _, err := revolveshell.PrepareSectorJoinPlans(input)
	require.NoError(t, err)
	gap := facetproof.SectorAngularGap(plans[0].Sector, plans[1].Sector)
	require.Greater(t, gap, 0.1)
	require.Less(t, gap, 0.9)

	// r_max is |contact| up to rounding; τ ≈ 0.55·gap makes the angle cost
	// about 1.1·gap while the seam cost stays below minRadius/2.
	rMax := sectorContact.Len()
	tilt := 0.55 * gap / (math.Pi / 2 * rMax)

	input.Tilted, input.FrameTilt = true, tilt
	_, _, err = revolveshell.PrepareSectorJoinPlans(input)
	require.ErrorContains(t, err, "root sectors are not separate")

	input.FrameTilt = tilt / 4
	_, _, err = revolveshell.PrepareSectorJoinPlans(input)
	require.NoError(t, err)

	input.FrameTilt = math.NaN()
	_, _, err = revolveshell.PrepareSectorJoinPlans(input)
	require.ErrorContains(t, err, "root sectors have no angular error bound")
}

package crossbore

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/orderedwork"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
)

// This file proves the band of the retained saddle chamfer's cutting tool
// (agent-docs/retained-edge-blend-design.md §4.9). The tool's band is the
// ruled surface of agent-docs/cross-cylinder-bore-chamfer-design.md, extended
// past both feet by BandExtension of its own length: at angle θ it runs from
// B⁺ = B − λ(O − B) to O⁺ = O + λ(O − B), O and B the outer and bore feet.

// BandExtension is λ, the fraction of the band's length the tool's band runs
// past each foot. It is a dyadic, so (1+λ)·O − λ·B is exact in rationals.
const BandExtension = 0.25

// ToolStations is the number of tool stations around the bore, at
// θ_j = 2πj/ToolStations. It is a multiple of four, so the quarter-turn
// mirror map of CellMirror holds for its cells too.
const ToolStations = 1024

// Pose maps the fixed pose (outer axis Z, bore axis X, bore axis at Z = 0,
// mouth at +X) into world coordinates exactly: world[Axis[i]] equals
// Sign[i]·local[i] + Shift[Axis[i]]. Sign entries are ±1 and Axis is a
// permutation, so a world point's local coordinates are exact differences.
type Pose struct {
	Axis  [3]int
	Sign  [3]float64
	Shift r3.Vec
}

// World maps a local point to world coordinates; the shift's addition rounds.
func (p Pose) World(local r3.Vec) r3.Vec {
	l := [3]float64{local.X, local.Y, local.Z}
	shift := [3]float64{p.Shift.X, p.Shift.Y, p.Shift.Z}
	var w [3]float64
	for i := range 3 {
		w[p.Axis[i]] = p.Sign[i]*l[i] + shift[p.Axis[i]]
	}
	return r3.Vec{X: w[0], Y: w[1], Z: w[2]}
}

// BandEnds returns the local band ends O⁺ and B⁺ of tool station j, in
// float: the feet of FeetAt with the bore axis at Z = 0, extended by
// BandExtension.
func BandEnds(R, r, setback float64, j int) (r3.Vec, r3.Vec) {
	theta := 2 * math.Pi * float64(j) / ToolStations
	o, b := FeetAt(R, r, 0, setback, theta)
	band := o.Sub(b)
	return o.Add(band.Scale(BandExtension)), b.Sub(band.Scale(BandExtension))
}

// ToolProof is the proven reading of the tool's band. Chord bounds the
// distance between each held band facet and the exact extended ruled surface
// under the pairing of agent-docs/retained-edge-blend-design.md §4.9, held
// rounding included. AreaLo and AreaHi enclose the exact area of the whole
// extended band.
type ToolProof struct {
	Chord, SampleError, AreaLo, AreaHi float64
}

// toolBase is one quarter-turn base cell's reading: the chord spread from the
// widened cell, the extended band ends' enclosures at the cell's own angle in
// the order O⁺ X, Y, Z, B⁺ X, Y, Z, and, for k < n/4, the enclosure of the
// band's area per radian over [θ_k, θ_k+1].
type toolBase struct {
	spread         float64
	ends           [6]crossIV
	areaLo, areaHi float64
}

// extend returns (1+λ)·a − λ·b, exactly.
func extend(a, b crossIV) crossIV {
	return crossSub(crossMul(crossPoint(1+BandExtension), a), crossMul(crossPoint(BandExtension), b))
}

// fiv is a float interval with outward rounding, for the band's area
// density: its inputs are the cell's rational enclosures read outward, and
// every operation widens by one ulp each way.
type fiv struct{ lo, hi float64 }

func down(x float64) float64 { return math.Nextafter(x, math.Inf(-1)) }
func up(x float64) float64   { return math.Nextafter(x, math.Inf(1)) }

func fivOf(a crossIV) fiv {
	return fiv{-proofbound.LazyRatFloatUp(proofarith.LazyNeg(a.Lo)), proofbound.LazyRatFloatUp(a.Hi)}
}
func (a fiv) add(b fiv) fiv { return fiv{down(a.lo + b.lo), up(a.hi + b.hi)} }
func (a fiv) sub(b fiv) fiv { return fiv{down(a.lo - b.hi), up(a.hi - b.lo)} }
func (a fiv) mul(b fiv) fiv {
	p := [4]float64{a.lo * b.lo, a.lo * b.hi, a.hi * b.lo, a.hi * b.hi}
	return fiv{down(min(p[0], p[1], p[2], p[3])), up(max(p[0], p[1], p[2], p[3]))}
}
func (a fiv) sq() fiv {
	switch {
	case a.lo >= 0:
		return fiv{down(a.lo * a.lo), up(a.hi * a.hi)}
	case a.hi <= 0:
		return fiv{down(a.hi * a.hi), up(a.lo * a.lo)}
	default:
		return fiv{0, up(max(a.lo*a.lo, a.hi*a.hi))}
	}
}

// norm encloses the length of an interval vector.
func norm(v [3]fiv) fiv {
	s := v[0].sq().add(v[1].sq()).add(v[2].sq())
	return fiv{down(math.Sqrt(max(s.lo, 0))), up(math.Sqrt(s.hi))}
}

func cross(a, b [3]fiv) [3]fiv {
	return [3]fiv{
		a[1].mul(b[2]).sub(a[2].mul(b[1])),
		a[2].mul(b[0]).sub(a[0].mul(b[2])),
		a[0].mul(b[1]).sub(a[1].mul(b[0])),
	}
}

// bandDensity encloses the extended band's area per radian over a cell. With
// F(θ, t) = B⁺(θ) + t·D(θ), D = O⁺ − B⁺, t ∈ [0, 1], the area element is
// |u + t(v − u)| with u = B⁺′ × D and v = O⁺′ × D, whose integral over t
// lies between |u + v|/2 and (|u| + |v|)/2.
func bandDensity(cell *CellExact) (float64, float64, bool) {
	var dO, dB, band [3]fiv
	for c := range 3 {
		dO[c] = fivOf(extend(cell.Derivs[c], cell.Derivs[3+c]))
		dB[c] = fivOf(extend(cell.Derivs[3+c], cell.Derivs[c]))
		band[c] = fivOf(crossMul(crossPoint(1+2*BandExtension), crossSub(cell.Coords[c], cell.Coords[3+c])))
	}
	u, v := cross(dB, band), cross(dO, band)
	var sum [3]fiv
	for c := range 3 {
		sum[c] = u[c].add(v[c])
	}
	nu, nv, ns := norm(u), norm(v), norm(sum)
	lo, hi := down(ns.lo/2), up(up(nu.hi+nv.hi)/2)
	if proofbound.IsNonFinite(lo) || proofbound.IsNonFinite(hi) || lo < 0 {
		return 0, 0, false
	}
	return lo, hi, true
}

// proveToolBase proves base cell k for the outer radius R, the bore r and the
// setback d. The widened cell runs CellExactProof's clearance, progress and
// fold tests and gives the derivative enclosures; a second call at the
// cell's own angle (no widening) gives tight enclosures of the feet there,
// and for k < n/4 a third over exactly [θ_k, θ_k+1] encloses the band's area.
func proveToolBase(R, r, d float64, k int, gap *big.Rat) (toolBase, error) {
	const n = ToolStations
	turn := big.NewRat(int64(k), n)
	cell, err := cellExactProofAt(R, r, d, turn, gap)
	if err != nil {
		return toolBase{}, err
	}
	point, err := cellExactProofAt(R, r, d, turn, new(big.Rat))
	if err != nil {
		return toolBase{}, err
	}
	var base toolBase
	spread := 0.0
	for c := range 3 {
		dO := extend(cell.Derivs[c], cell.Derivs[3+c])
		dB := extend(cell.Derivs[3+c], cell.Derivs[c])
		spread = max(spread, proofbound.LazyIntervalAbsUpperUp(crossSub(dB, dO)),
			crossWidth(dO), crossWidth(dB))
		base.ends[c] = extend(point.Coords[c], point.Coords[3+c])
		base.ends[3+c] = extend(point.Coords[3+c], point.Coords[c])
	}
	base.spread = spread
	if k < n/4 {
		half := new(big.Rat).Quo(proofbound.PiUpper, big.NewRat(n, 1))
		mid, err := cellExactProofAt(R, r, d, big.NewRat(int64(2*k+1), 2*n), half)
		if err != nil {
			return toolBase{}, err
		}
		var ok bool
		if base.areaLo, base.areaHi, ok = bandDensity(&mid); !ok {
			return toolBase{}, fmt.Errorf("%w: the saddle chamfer's band has no area enclosure", decaderr.ErrUnsupported)
		}
	}
	return base, nil
}

// worldError compares a held world point with a local enclosure triple
// mirrored by flipY and flipZ and mapped by pose. Negation and the shift are
// exact, so only the reading's own rounding enters, upward.
func worldError(held r3.Vec, local [3]crossIV, flipY, flipZ bool, pose Pose) float64 {
	if flipY {
		local[1] = crossNeg(local[1])
	}
	if flipZ {
		local[2] = crossNeg(local[2])
	}
	h := [3]float64{held.X, held.Y, held.Z}
	shift := [3]float64{pose.Shift.X, pose.Shift.Y, pose.Shift.Z}
	e := 0.0
	for i := range 3 {
		v := local[i]
		if pose.Sign[i] < 0 {
			v = crossNeg(v)
		}
		a := pose.Axis[i]
		e = max(e, proofbound.LazyIntervalFloatError(crossAdd(v, crossPoint(shift[a])), h[a]))
	}
	return e
}

// ProveToolBand proves the tool band for the outer radius R, the bore r and
// the setback d. It proves the ToolStations/4 + 1 quarter-turn base cells with
// CellExactProof (every failed interval sign refuses), and compares held(j),
// the world band ends O⁺ and B⁺ of every station j, with its base cell's
// mirrored point enclosures (cellMirrorOf, which holds for the fixed pose
// exactly). The chord term pairs a point of a held band triangle with the
// exact band at the same parameters: on either triangle of the quad between
// stations j and j+1, the difference is at most h·u(1−u) times the spread
// of the end curves' θ-derivatives, u ∈ [0, 1], so h/4 times it, with h the
// station step.
func ProveToolBand(ctx context.Context, R, r, d float64, pose Pose,
	held func(j int) (r3.Vec, r3.Vec)) (ToolProof, error) {
	const n = ToolStations
	gap := new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(2, 1), proofbound.PiUpper), big.NewRat(n, 1))
	bases := make([]toolBase, n/4+1)
	workers := orderedwork.EachWorkers(len(bases), CellParallelMin)
	if err := cellEach(ctx, len(bases), workers, func(k int) error {
		b, err := proveToolBase(R, r, d, k, gap)
		bases[k] = b
		return err
	}); err != nil {
		return ToolProof{}, err
	}
	samples := make([]float64, n)
	if err := cellEach(ctx, n, workers, func(j int) error {
		k, flipY, flipZ := cellMirrorOf(j, n)
		o, b := held(j)
		ends := &bases[k].ends
		samples[j] = max(
			worldError(o, [3]crossIV{ends[0], ends[1], ends[2]}, flipY, flipZ, pose),
			worldError(b, [3]crossIV{ends[3], ends[4], ends[5]}, flipY, flipZ, pose))
		return nil
	}); err != nil {
		return ToolProof{}, err
	}
	if err := ctx.Err(); err != nil {
		return ToolProof{}, err
	}
	var p ToolProof
	spread := 0.0
	areaLo, areaHi := 0.0, 0.0
	for k, b := range bases {
		spread = max(spread, b.spread)
		if k < n/4 {
			areaLo, areaHi = down(areaLo+b.areaLo), up(areaHi+b.areaHi)
		}
	}
	for _, e := range samples {
		p.SampleError = max(p.SampleError, e)
	}
	// The four quarter turns are mirror images, so the band's whole area is
	// four times the quarter's; each cell spans 2π/n.
	stepLo := proofbound.RatFloatDown(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiLower), big.NewRat(n, 1)))
	stepHi := proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiUpper), big.NewRat(n, 1)))
	p.AreaLo, p.AreaHi = down(areaLo*stepLo), up(areaHi*stepHi)
	step := proofbound.RatFloatUp(gap)
	p.SampleError = proofbound.Radius3D(p.SampleError)
	p.Chord = proofbound.AbsSumUpper(
		proofbound.Radius3D(proofbound.DivUpper(proofbound.ProductUpper(step, spread), 4)), p.SampleError)
	if proofbound.IsNonFinite(p.Chord) || proofbound.IsNonFinite(p.AreaHi) || !(p.AreaLo >= 0) {
		return ToolProof{}, fmt.Errorf("%w: the saddle chamfer's band has no finite bound", decaderr.ErrUnsupported)
	}
	return p, nil
}

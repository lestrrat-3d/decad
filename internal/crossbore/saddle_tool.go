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

// ToolRows is the number of rows the band is split into across its length,
// at t_i = i/ToolRows from B⁺ (t = 0) to O⁺ (t = 1). It is a power of two,
// so every t_i is a dyadic, and no t_i is 1/6 or 5/6, where the exact band
// meets the bore and the shaft (BandExtension = 1/4).
const ToolRows = 8

// ToolAreaCells is the number of cells each quad's area enclosure splits
// into along θ. An interval enclosure over a cell is wider than the range it
// holds by a part proportional to the cell's width, so the band's area
// interval narrows in proportion to the split.
const ToolAreaCells = 8

// BandRow returns the held point of row i between the local band ends o
// (O⁺) and b (B⁺): b + t_i·(o − b), in float.
func BandRow(o, b r3.Vec, i int) r3.Vec {
	switch i {
	case 0:
		return b
	case ToolRows:
		return o
	}
	return b.Add(o.Sub(b).Scale(float64(i) / ToolRows))
}

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
// rounding included. RowLo[i] and RowHi[i] enclose the exact area of row i of
// the extended band, t ∈ [t_i, t_i+1]; AreaLo and AreaHi are their sums, the
// whole band's.
type ToolProof struct {
	Chord, SampleError, AreaLo, AreaHi float64
	RowLo, RowHi                       [ToolRows]float64
}

// toolBase is one quarter-turn base cell's reading: the extended band ends'
// enclosures at the cell's own angle in the order O⁺ X, Y, Z, B⁺ X, Y, Z,
// and, for k < n/4, from the cell over exactly [θ_k, θ_k+1], the chord
// spread of that quad and the enclosure of each row's area per radian.
type toolBase struct {
	spread         float64
	ends           [6]crossIV
	areaLo, areaHi [ToolRows]float64
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

// bandDensity encloses each row's area per radian over a cell. With
// F(θ, t) = B⁺(θ) + t·D(θ), D = O⁺ − B⁺, the area element is |w(t)| with
// w(t) = (1 − t)·u + t·v, u = B⁺′ × D and v = O⁺′ × D. |w| is convex in t, so
// its integral over a row [a, b] lies between (b − a)·|w((a + b)/2)| (Jensen)
// and (b − a)·(|w(a)| + |w(b)|)/2 (the chord above a convex function). The
// interval (1 − t)·U + t·V holds w(t) at every θ of the cell, U and V
// holding u and v there.
func bandDensity(cell *CellExact) ([ToolRows]float64, [ToolRows]float64, bool) {
	var dO, dB, band [3]fiv
	for c := range 3 {
		dO[c] = fivOf(extend(cell.Derivs[c], cell.Derivs[3+c]))
		dB[c] = fivOf(extend(cell.Derivs[3+c], cell.Derivs[c]))
		band[c] = fivOf(crossMul(crossPoint(1+2*BandExtension), crossSub(cell.Coords[c], cell.Coords[3+c])))
	}
	u, v := cross(dB, band), cross(dO, band)
	// at encloses |w(t)| for the dyadic t = num/(2·ToolRows); the scalings
	// by a power of two and by num are rounded outward by mul.
	at := func(num int) fiv {
		t := float64(num) / (2 * ToolRows)
		var w [3]fiv
		for c := range 3 {
			w[c] = u[c].mul(fiv{1 - t, 1 - t}).add(v[c].mul(fiv{t, t}))
		}
		return norm(w)
	}
	var lo, hi [ToolRows]float64
	const k = 1.0 / ToolRows
	for i := range ToolRows {
		a, mid, b := at(2*i), at(2*i+1), at(2*i+2)
		lo[i], hi[i] = down(mid.lo*k), up(up(up(a.hi+b.hi)/2)*k)
		if proofbound.IsNonFinite(lo[i]) || proofbound.IsNonFinite(hi[i]) || lo[i] < 0 {
			return lo, hi, false
		}
	}
	return lo, hi, true
}

// quadSpread is the chord spread of the quad a cell spans: per coordinate,
// the larger width of the band ends' θ-derivative enclosures plus the
// largest |D′| divided by ToolRows, D′ = (1 + 2λ)·(O′ − B′); the largest
// over the three coordinates (agent-docs/retained-edge-blend-design.md §4.9,
// proof 2).
func quadSpread(cell *CellExact) float64 {
	spread := 0.0
	for c := range 3 {
		dO := extend(cell.Derivs[c], cell.Derivs[3+c])
		dB := extend(cell.Derivs[3+c], cell.Derivs[c])
		twist := proofbound.LazyIntervalAbsUpperUp(
			crossMul(crossPoint(1+2*BandExtension), crossSub(cell.Derivs[c], cell.Derivs[3+c])))
		spread = max(spread, proofbound.AbsSumUpper(max(crossWidth(dO), crossWidth(dB)),
			proofbound.DivUpper(twist, ToolRows)))
	}
	return spread
}

// proveToolBase proves base cell k for the outer radius R, the bore r and the
// setback d. The widened cell runs CellExactProof's clearance, progress and
// fold tests; a second call at the cell's own angle (no widening) gives tight
// enclosures of the feet there, and for k < n/4 a third over exactly
// [θ_k, θ_k+1] gives that quad's chord spread and encloses each row's area.
func proveToolBase(R, r, d float64, k int, gap *big.Rat) (toolBase, error) {
	const n = ToolStations
	turn := big.NewRat(int64(k), n)
	if _, err := cellExactProofAt(R, r, d, turn, gap); err != nil {
		return toolBase{}, err
	}
	point, err := cellExactProofAt(R, r, d, turn, new(big.Rat))
	if err != nil {
		return toolBase{}, err
	}
	var base toolBase
	for c := range 3 {
		base.ends[c] = extend(point.Coords[c], point.Coords[3+c])
		base.ends[3+c] = extend(point.Coords[3+c], point.Coords[c])
	}
	if k < n/4 {
		half := new(big.Rat).Quo(proofbound.PiUpper, big.NewRat(n, 1))
		mid, err := cellExactProofAt(R, r, d, big.NewRat(int64(2*k+1), 2*n), half)
		if err != nil {
			return toolBase{}, err
		}
		base.spread = quadSpread(&mid)
		const q = ToolAreaCells
		sub := new(big.Rat).Quo(proofbound.PiUpper, big.NewRat(n*q, 1))
		for p := range q {
			cell, err := cellExactProofAt(R, r, d, big.NewRat(int64(2*(k*q+p)+1), 2*n*q), sub)
			if err != nil {
				return toolBase{}, err
			}
			lo, hi, ok := bandDensity(&cell)
			if !ok {
				return toolBase{}, fmt.Errorf("%w: the saddle chamfer's band has no area enclosure", decaderr.ErrUnsupported)
			}
			for i := range ToolRows {
				base.areaLo[i], base.areaHi[i] = down(base.areaLo[i]+lo[i]), up(base.areaHi[i]+hi[i])
			}
		}
	}
	return base, nil
}

// rowEnclosure returns row i's exact point (1 − t_i)·B⁺ + t_i·O⁺ from the
// band ends' enclosures in the order O⁺ X, Y, Z, B⁺ X, Y, Z.
func rowEnclosure(ends *[6]crossIV, i int) [3]crossIV {
	t := big.NewRat(int64(i), ToolRows)
	s := new(big.Rat).Sub(big.NewRat(1, 1), t)
	var p [3]crossIV
	for c := range 3 {
		p[c] = crossAdd(crossMul(proofarith.LazyPoint(s), ends[3+c]), crossMul(proofarith.LazyPoint(t), ends[c]))
	}
	return p
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
// CellExactProof (every failed interval sign refuses), and compares held(j, i),
// the world band point of station j on row i (row 0 is B⁺, row ToolRows is
// O⁺), with its base cell's mirrored point enclosures (cellMirrorOf, which
// holds for the fixed pose exactly). The chord term pairs a point of a held
// band triangle with the exact band at the same parameters: on either
// triangle of the quad between stations j and j+1 and rows i and i+1, each
// coordinate of the difference is at most h·u(1−u) times the quad's spread
// (quadSpread), u ∈ [0, 1] the fraction of the station step h, so h/4 times
// it (agent-docs/retained-edge-blend-design.md §4.9, proof 2).
func ProveToolBand(ctx context.Context, R, r, d float64, pose Pose,
	held func(j, i int) r3.Vec) (ToolProof, error) {
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
		ends := &bases[k].ends
		for i := range ToolRows + 1 {
			samples[j] = max(samples[j], worldError(held(j, i), rowEnclosure(ends, i), flipY, flipZ, pose))
		}
		return nil
	}); err != nil {
		return ToolProof{}, err
	}
	if err := ctx.Err(); err != nil {
		return ToolProof{}, err
	}
	var p ToolProof
	spread := 0.0
	var areaLo, areaHi [ToolRows]float64
	for _, b := range bases[:n/4] {
		spread = max(spread, b.spread)
		for i := range ToolRows {
			areaLo[i], areaHi[i] = down(areaLo[i]+b.areaLo[i]), up(areaHi[i]+b.areaHi[i])
		}
	}
	for _, e := range samples {
		p.SampleError = max(p.SampleError, e)
	}
	// The four quarter turns are mirror images, so each row's whole area is
	// four times the quarter's; each area cell spans 2π/(n·ToolAreaCells).
	// Every quad of the turn is the mirror image of one of the quarter's, so
	// its spread is too.
	cells := big.NewRat(n*ToolAreaCells, 1)
	stepLo := proofbound.RatFloatDown(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiLower), cells))
	stepHi := proofbound.RatFloatUp(new(big.Rat).Quo(new(big.Rat).Mul(big.NewRat(8, 1), proofbound.PiUpper), cells))
	for i := range ToolRows {
		p.RowLo[i], p.RowHi[i] = down(areaLo[i]*stepLo), up(areaHi[i]*stepHi)
		p.AreaLo, p.AreaHi = down(p.AreaLo+p.RowLo[i]), up(p.AreaHi+p.RowHi[i])
	}
	step := proofbound.RatFloatUp(gap)
	p.SampleError = proofbound.Radius3D(p.SampleError)
	p.Chord = proofbound.AbsSumUpper(
		proofbound.Radius3D(proofbound.DivUpper(proofbound.ProductUpper(step, spread), 4)), p.SampleError)
	if proofbound.IsNonFinite(p.Chord) || proofbound.IsNonFinite(p.AreaHi) || !(p.AreaLo >= 0) {
		return ToolProof{}, fmt.Errorf("%w: the saddle chamfer's band has no finite bound", decaderr.ErrUnsupported)
	}
	return p, nil
}

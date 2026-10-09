package decad

import (
	"context"
	"fmt"
	"math"
	"math/big"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// partialBandMesh carries the two end columns of each selected straight walk.
// Adjacent walks share their vertex columns through the mesh's coordinate key.
type partialBandMesh struct {
	walks    []partialBandWalk
	columns  []partialBandColumn
	cells    []filletCell
	patchSag []float64
	arcCols  map[int][]int
}

type partialBandWalk struct {
	index int
	cols  [2]int
}

type partialBandColumn struct {
	side, cap Point2
	capBound  proofbound.WalkEndBound
}

func chordPartialFilletBand(ctx context.Context, bp brepPayload, b brepLoopBand, f brepFace,
	cbp capBlendPayload, chord float64) (brepBandChord, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	side, err := oneLoopCornerLoop(budget, b.orig, work)
	if err != nil {
		return brepBandChord{}, err
	}
	if len(side.walks) != len(b.selected) {
		return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band's side walks disagree with its selection`, ErrUnsupported)
	}
	cap := f.regionLoop(b.loop)
	partial := &partialBandMesh{arcCols: map[int][]int{}}
	walkCols := make([][2]int, len(b.selected))
	for i, on := range b.selected {
		if !on {
			continue
		}
		if b.capWalk[i] < 0 || b.capWalk[i] >= len(cap.Segments) {
			return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band's selected cap segment is missing`, ErrUnsupported)
		}
		s := side.walks[i]
		c, err := boundarywalk.WalkOf(cap.Segments[b.capWalk[i]], work)
		if err != nil {
			return brepBandChord{}, err
		}
		if !s.IsLine() || !c.IsLine() {
			return brepBandChord{}, fmt.Errorf(`%w: a selected partial fillet walk is curved`, ErrUnsupported)
		}
		walkCols[i] = [2]int{len(partial.columns), len(partial.columns) + 1}
		partial.columns = append(partial.columns,
			partialBandColumn{side: Point2{U: s.StartU, V: s.StartV}, cap: Point2{U: c.StartU, V: c.StartV}},
			partialBandColumn{side: Point2{U: s.EndU, V: s.EndV}, cap: Point2{U: c.EndU, V: c.EndV}})
		partial.walks = append(partial.walks, partialBandWalk{index: i, cols: walkCols[i]})
	}
	if len(partial.walks) == 0 {
		return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band selects no walks`, ErrUnsupported)
	}
	for _, w := range partial.walks {
		i := w.index
		partial.cells = append(partial.cells, filletCell{c0: w.cols[0], c1: w.cols[1], patch: len(partial.patchSag)})
		partial.patchSag = append(partial.patchSag, 0)
		k := (i + 1) % len(b.selected)
		if b.capArc[k] < 0 {
			continue
		}
		if b.capArc[k] >= len(cap.Segments) || !b.selected[k] {
			return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band's reflex connector is missing`, ErrUnsupported)
		}
		seg := cap.Segments[b.capArc[k]]
		arc, err := boundarywalk.WalkOf(seg, work)
		if err != nil {
			return brepBandChord{}, err
		}
		if !arc.IsCircular() || arc.Closed {
			return brepBandChord{}, fmt.Errorf(`%w: a partial fillet's connector is not an open arc`, ErrUnsupported)
		}
		count, sag, err := tessellation.ChordCount(arc, chord/2, 1)
		if err != nil {
			return brepBandChord{}, err
		}
		cols := []int{walkCols[i][1]}
		corner := side.walks[k]
		for j := 1; j < count; j++ {
			theta := arc.Th0 + (arc.Th1-arc.Th0)*float64(j)/float64(count)
			p := Point2{U: arc.CU + arc.Radius*math.Cos(theta),
				V: arc.CV + arc.Radius*math.Sin(theta)}
			cols = append(cols, len(partial.columns))
			partial.columns = append(partial.columns, partialBandColumn{
				side: Point2{U: corner.StartU, V: corner.StartV}, cap: p,
				capBound: stationbound.ChordStationBound(seg, j, count, p.U, p.V)})
		}
		cols = append(cols, walkCols[k][0])
		partial.arcCols[b.capArc[k]] = cols
		for j := 0; j < len(cols)-1; j++ {
			partial.cells = append(partial.cells, filletCell{c0: cols[j], c1: cols[j+1], patch: len(partial.patchSag)})
		}
		partial.patchSag = append(partial.patchSag, sag)
	}
	sideZ, sideDelta := b.sideLevel(f)
	return brepBandChord{band: b, face: f, cbp: cbp, start: b.matSign(f) > 0,
		sideZ: sideZ, sideDelta: sideDelta,
		fillet: &filletRings{n: filletRingCount(b.setback.axialUpper(), chord)}, partial: partial}, nil
}

func (bc *brepBandChord) placePartial(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	for _, col := range bc.partial.columns {
		s, c := col.side, col.cap
		bc.sideV = append(bc.sideV, addVertex(e.Canon(s.U, s.V, bc.sideZ), proofbound.WalkEndBound{}))
		bc.capV = append(bc.capV, addVertex(e.Canon(c.U, c.V, bc.face.z0), col.capBound))
	}
}

func (bc *brepBandChord) placePartialRings(e brepEmbed,
	addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	fr := bc.fillet
	n, N := fr.n, len(bc.capV)
	fr.ringV = make([][]int, n+1)
	fr.dev = make([][]float64, n+1)
	fr.ringV[0], fr.ringV[n] = bc.sideV, bc.capV
	r, rd := bc.band.setback.dc, bc.band.setback.dcDelta
	rz0, rzd := proofarith.FloatRat(bc.face.z0), proofarith.FloatRat(bc.face.z0Delta)
	rr, rrd := proofarith.FloatRat(r), proofarith.FloatRat(rd)
	if rz0 == nil || rzd == nil || rr == nil || rrd == nil {
		return fmt.Errorf(`%w: a partial fillet band's radius or level is not finite`, ErrNotFinite)
	}
	rIv := proofbound.IntervalWiden(proofbound.PointInterval(rr), rrd)
	one := proofbound.PointInterval(big.NewRat(1, 1))
	m := bc.band.matSign(bc.face)
	for k := 1; k < n; k++ {
		phi := float64(k) * (math.Pi / 2) / float64(n)
		sinIv, cosIv, ok := proofbound.RadSinCosInterval(proofarith.FloatRat(phi))
		if !ok {
			return fmt.Errorf(`%w: a partial fillet ring's angle has no sine enclosure`, ErrUnsupported)
		}
		fraction := 1 - math.Cos(phi)
		fRat := proofarith.FloatRat(fraction)
		errF := proofbound.IntervalFloatError(proofbound.IntervalSub(one, cosIv), fraction)
		z := bc.sideZ - m*r*math.Sin(phi)
		zIv := proofbound.IntervalWiden(proofbound.IntervalAdd(proofbound.PointInterval(rz0),
			proofbound.IntervalScale(proofbound.IntervalMul(rIv, proofbound.IntervalSub(one, sinIv)),
				big.NewRat(int64(m), 1))), rzd)
		errZ := proofbound.IntervalFloatError(zIv, z)
		if fRat == nil || proofbound.IsNonFinite(errF) || proofbound.IsNonFinite(errZ) {
			return fmt.Errorf(`%w: a partial fillet ring's position is not finite`, ErrNotFinite)
		}
		fr.ringV[k] = make([]int, N)
		fr.dev[k] = make([]float64, N)
		for c := range N {
			col := bc.partial.columns[c]
			s, cap := col.side, col.cap
			u, ru, okU := interpolate(s.U, cap.U, fraction, fRat)
			v, rv, okV := interpolate(s.V, cap.V, fraction, fRat)
			if !okU || !okV {
				return fmt.Errorf(`%w: a partial fillet ring's position is not finite`, ErrNotFinite)
			}
			span := proofbound.AbsSumUpper(math.Abs(cap.U-s.U), math.Abs(cap.V-s.V))
			dev := proofbound.AbsSumUpper(ru, rv, proofbound.ProductUpper(errF, span), errZ)
			fr.dev[k][c] = dev
			fr.ringV[k][c] = addVertex(e.Canon(u, v, z), proofbound.WalkEndBound{U: dev, V: dev})
		}
	}
	return nil
}

// partialOpenPolys gives terminal meridians and cap connector arcs the same
// stations as their adjacent patches, in each record face's walk direction.
func (bc *brepBandChord) partialOpenPolys(topo *brepTopology, canon [][3]float64) (map[int][]int, error) {
	out := map[int][]int{}
	for _, t := range bc.band.terminals {
		ui := -1
		for _, candidate := range topo.faceUses[t.face] {
			u := topo.uses[candidate]
			if u.Part == brepLoopSeg && u.Loop == t.loop && u.Seg == t.seg {
				ui = candidate
				break
			}
		}
		if ui < 0 {
			return nil, fmt.Errorf(`%w: a partial fillet's terminal has no face use`, ErrUnsupported)
		}
		u := topo.uses[ui]
		for c := range bc.capV {
			lo, hi := canon[bc.fillet.ringV[0][c]], canon[bc.fillet.ringV[bc.fillet.n][c]]
			if u.From != lo || u.To != hi {
				if u.From != hi || u.To != lo {
					continue
				}
			}
			poly := make([]int, bc.fillet.n+1)
			for k := range poly {
				poly[k] = bc.fillet.ringV[k][c]
			}
			if u.From == hi {
				slices.Reverse(poly)
			}
			out[ui] = poly
			break
		}
		if out[ui] == nil {
			return nil, fmt.Errorf(`%w: a partial fillet's terminal arc has no matching meridian`, ErrUnsupported)
		}
	}
	for seg, cols := range bc.partial.arcCols {
		ui := -1
		for _, candidate := range topo.faceUses[bc.band.face] {
			u := topo.uses[candidate]
			if u.Part == brepLoopSeg && u.Loop == bc.band.loop && u.Seg == seg {
				ui = candidate
				break
			}
		}
		if ui < 0 {
			return nil, fmt.Errorf(`%w: a partial fillet's cap connector has no face use`, ErrUnsupported)
		}
		poly := make([]int, len(cols))
		for j, col := range cols {
			poly[j] = bc.fillet.ringV[bc.fillet.n][col]
		}
		u := topo.uses[ui]
		switch {
		case canon[poly[0]] == u.From && canon[poly[len(poly)-1]] == u.To:
		case canon[poly[0]] == u.To && canon[poly[len(poly)-1]] == u.From:
			slices.Reverse(poly)
		default:
			return nil, fmt.Errorf(`%w: a partial fillet's cap connector disagrees with its samples`, ErrUnsupported)
		}
		out[ui] = poly
	}
	return out, nil
}

func (bc *brepBandChord) emitPartialFillet(m *Mesh, faceOfRole func(string) (*Face, error),
	bump func(*Face, float64)) error {
	fr := bc.fillet
	fr.patchEps = map[*Face]float64{}
	r := bc.band.setback.axialUpper()
	dphi := proofbound.UpRound(proofbound.UpRound(math.Pi/2*(1+1e-12)) / float64(fr.n))
	sPhi := proofbound.ProductUpper(r, proofbound.UpRound(proofbound.ProductUpper(dphi, dphi)/8))
	delta := bc.face.delta
	levelDelta := proofbound.AbsSumUpper(bc.band.setback.dsDelta, bc.sideDelta)
	axial := bc.cbp.capBandLevel(bc.face.z0, bc.band.matSign(bc.face)).Bound
	for p := range bc.partial.patchSag {
		face, err := faceOfRole(bc.band.patchRole(p))
		if err != nil {
			return err
		}
		first := len(m.triangles)
		twist := 0.0
		tri := func(a, b, c int) {
			if a != b && b != c && a != c {
				m.addTriangle([3]int{a, b, c}, face)
			}
		}
		for _, cell := range bc.partial.cells {
			if cell.patch != p {
				continue
			}
			for k := range fr.n {
				a, b := fr.ringV[k][cell.c0], fr.ringV[k][cell.c1]
				A, B := fr.ringV[k+1][cell.c0], fr.ringV[k+1][cell.c1]
				if bc.start {
					tri(A, B, b)
					tri(A, b, a)
				} else {
					tri(a, b, B)
					tri(a, B, A)
				}
				if a != b {
					twist = math.Max(twist, proofbound.CellTwistOffsetUpper(
						m.vertices[a], m.vertices[b], m.vertices[A], m.vertices[B]))
				}
			}
		}
		eps := proofbound.AbsSumUpper(sPhi, bc.partial.patchSag[p], twist)
		bc.finishPatch(m, face, eps, delta, levelDelta, axial, first, len(m.triangles), bump)
	}
	return nil
}

func (bc *brepBandChord) partialMotion(motion, store, round []float64) {
	for _, vi := range bc.sideV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], bc.sideDelta))
	}
	for _, vi := range bc.capV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], bc.face.delta, bc.face.z0Delta))
	}
	for k := 1; k < bc.fillet.n; k++ {
		for c, vi := range bc.fillet.ringV[k] {
			ends := math.Max(motion[bc.fillet.ringV[0][c]], motion[bc.fillet.ringV[bc.fillet.n][c]])
			motion[vi] = proofbound.AbsSumUpper(ends, bc.fillet.dev[k][c], round[vi])
		}
	}
}

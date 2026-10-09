package decad

import (
	"context"
	"fmt"
	"math"
	"slices"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/freeform"
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

func chordPartialFilletBand(ctx context.Context, b brepLoopBand, f brepFace,
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
	contour := f.regionLoop(b.loop)
	partial := &partialBandMesh{arcCols: map[int][]int{}}
	walkCols := make([][2]int, len(b.selected))
	for i, on := range b.selected {
		if !on {
			continue
		}
		if b.capWalk[i] < 0 || b.capWalk[i] >= len(contour.Segments) {
			return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band's selected cap segment is missing`, ErrUnsupported)
		}
		s := side.walks[i]
		c, err := boundarywalk.WalkOf(contour.Segments[b.capWalk[i]], work)
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
		partial.cells = append(partial.cells, filletCell{C0: w.cols[0], C1: w.cols[1], Patch: len(partial.patchSag)})
		partial.patchSag = append(partial.patchSag, 0)
		k := (i + 1) % len(b.selected)
		if b.capArc[k] < 0 {
			continue
		}
		if b.capArc[k] >= len(contour.Segments) || !b.selected[k] {
			return brepBandChord{}, fmt.Errorf(`%w: a partial fillet band's reflex connector is missing`, ErrUnsupported)
		}
		seg := contour.Segments[b.capArc[k]]
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
		for j := range len(cols) - 1 {
			partial.cells = append(partial.cells, filletCell{C0: cols[j], C1: cols[j+1], Patch: len(partial.patchSag)})
		}
		partial.patchSag = append(partial.patchSag, sag)
	}
	sideZ, sideDelta := b.sideLevel(f)
	return brepBandChord{band: b, face: f, cbp: cbp, start: b.matSign(f) > 0,
		sideZ: sideZ, sideDelta: sideDelta,
		fillet: &filletRings{n: tessellation.FilletRingCount(b.setback.axialUpper(), chord)}, partial: partial}, nil
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
	side, capPoints := make([]Point2, N), make([]Point2, N)
	for c, col := range bc.partial.columns {
		side[c], capPoints[c] = col.side, col.cap
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: side, Cap: capPoints, Radius: bc.band.setback.dc,
		RadiusDelta: bc.band.setback.dcDelta, CapLevel: bc.face.z0,
		CapLevelDelta: bc.face.z0Delta, SideLevel: bc.sideZ,
		MaterialSign: bc.band.matSign(bc.face), Count: n,
		Noun: "a partial fillet band", RingNoun: "a partial fillet",
	})
	if err != nil {
		return err
	}
	for k := 1; k < n; k++ {
		fr.ringV[k] = make([]int, N)
		fr.dev[k] = make([]float64, N)
		for c, point := range points[k] {
			fr.dev[k][c] = point.Delta
			fr.ringV[k][c] = addVertex(e.Canon(point.Point.U, point.Point.V, point.Z),
				proofbound.WalkEndBound{U: point.Delta, V: point.Delta})
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
			if cell.Patch != p {
				continue
			}
			for k := range fr.n {
				a, b := fr.ringV[k][cell.C0], fr.ringV[k][cell.C1]
				A, B := fr.ringV[k+1][cell.C0], fr.ringV[k+1][cell.C1]
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

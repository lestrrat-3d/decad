package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
)

// chordPartialFilletBand adapts the recorded band to its internal sample layout.
func chordPartialFilletBand(ctx context.Context, b brepLoopBand, f brepFace,
	cbp capBlendPayload, chord float64) (brepBandChord, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	side, err := oneLoopCornerLoop(budget, b.orig, work)
	if err != nil {
		return brepBandChord{}, err
	}
	contour := f.regionLoop(b.loop)
	partial, err := partialband.PartialFilletLayout(side.walks, b.orig.Segments,
		contour.Segments, b.selected, b.capWalk, b.capArc, b.setback.dc, chord, work)
	if err != nil {
		return brepBandChord{}, err
	}
	sideZ, sideDelta := b.sideLevel(f)
	return brepBandChord{band: b, face: f, cbp: cbp, start: b.matSign(f) > 0,
		sideZ: sideZ, sideDelta: sideDelta,
		fillet: &filletRings{n: tessellation.FilletRingCount(b.setback.axialUpper(), chord)}, partial: partial}, nil
}

func (bc *brepBandChord) placePartial(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	for _, col := range bc.partial.Columns {
		s, c := col.Side, col.Cap
		bc.sideV = append(bc.sideV, addVertex(e.Canon(s.U, s.V, bc.sideZ), col.SideBound))
		bc.capV = append(bc.capV, addVertex(e.Canon(c.U, c.V, bc.face.z0), col.CapBound))
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
	for c, col := range bc.partial.Columns {
		side[c], capPoints[c] = col.Side, col.Cap
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: point2ToRecordSlice(side), Cap: point2ToRecordSlice(capPoints), Radius: bc.band.setback.dc,
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

// partialOpenPolys adapts recorded B-rep uses to the internal band layout.
func (bc *brepBandChord) partialOpenPolys(topo *brepTopology, canon [][3]float64) (map[int][]int, error) {
	terminals := make([]partialband.Terminal, len(bc.band.terminals))
	for i, terminal := range bc.band.terminals {
		terminals[i] = partialband.Terminal{Face: terminal.face, Loop: terminal.loop, Seg: terminal.seg}
	}
	ringV := [][]int{bc.sideV, bc.capV}
	if bc.fillet != nil {
		ringV = bc.fillet.ringV
	}
	return partialband.OpenPolys(partialband.OpenPolyInput{
		Layout: bc.partial, Face: bc.band.face, Loop: bc.band.loop,
		Orig: bc.band.orig.Segments, CapWalk: bc.band.capWalk, Terminals: terminals,
		RingV: ringV, Canon: canon, Uses: topo.uses, FaceUses: topo.faceUses,
		Open: topo.open, Embed: topo.embeds[bc.band.face], SideZ: bc.sideZ,
	})
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
	for p := range bc.partial.PatchSag {
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
		for _, cell := range bc.partial.Cells {
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
		eps := proofbound.AbsSumUpper(sPhi, bc.partial.PatchSag[p], twist)
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
	if bc.fillet == nil {
		return
	}
	for k := 1; k < bc.fillet.n; k++ {
		for c, vi := range bc.fillet.ringV[k] {
			ends := math.Max(motion[bc.fillet.ringV[0][c]], motion[bc.fillet.ringV[bc.fillet.n][c]])
			motion[vi] = proofbound.AbsSumUpper(ends, bc.fillet.dev[k][c], round[vi])
		}
	}
}

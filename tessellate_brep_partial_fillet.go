package decad

import (
	"context"
	"math"

	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// chordPartialFilletBand adapts the recorded band to its internal sample layout.
func chordPartialFilletBand(ctx context.Context, b brepLoopBand, f brepFace,
	cbp capBlendPayload, chord float64) (brepBandChord, error) {
	budget := proofbound.NewWorkBudget(ctx)
	work := freeform.NewFreeformWork()
	side, err := capblend.OneLoopCornerLoop(budget, b.Orig, work)
	if err != nil {
		return brepBandChord{}, err
	}
	contour := f.RegionLoop(b.Loop)
	partial, err := partialband.PartialFilletLayout(side.Walks, b.Orig.Segments,
		contour.Segments, b.Selected, b.CapWalk, b.CapArc, b.Setback.Dc, chord, work)
	if err != nil {
		return brepBandChord{}, err
	}
	sideZ, sideDelta := b.SideLevel(f)
	return brepBandChord{band: b, face: f, cbp: cbp, start: b.MatSign(f) > 0,
		sideZ: sideZ, sideDelta: sideDelta,
		fillet: &filletRings{n: tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)}, partial: partial}, nil
}

func (bc *brepBandChord) placePartial(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	for _, col := range bc.partial.Columns {
		s, c := col.Side, col.Cap
		bc.sideV = append(bc.sideV, addVertex(e.Canon(s.U, s.V, bc.sideZ), col.SideBound))
		bc.capV = append(bc.capV, addVertex(e.Canon(c.U, c.V, bc.face.Z0), col.CapBound))
	}
}

func (bc *brepBandChord) placePartialRings(e brepEmbed,
	addVertex func([3]float64, proofbound.WalkEndBound) int) error {
	fr := bc.fillet
	n, N := fr.n, len(bc.capV)
	fr.ringV = make([][]int, n+1)
	fr.dev = make([][]float64, n+1)
	fr.ringV[0], fr.ringV[n] = bc.sideV, bc.capV
	side, capPoints := make([]planePoint, N), make([]planePoint, N)
	for c, col := range bc.partial.Columns {
		side[c], capPoints[c] = col.Side, col.Cap
	}
	points, err := tessellation.FilletRingGeometry(tessellation.FilletRingInput{
		Side: point2ToRecordSlice(side), Cap: point2ToRecordSlice(capPoints), Radius: bc.band.Setback.Dc,
		RadiusDelta: bc.band.Setback.DcDelta, CapLevel: bc.face.Z0,
		CapLevelDelta: bc.face.Z0Delta, SideLevel: bc.sideZ,
		MaterialSign: bc.band.MatSign(bc.face), Count: n,
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
func (bc *brepBandChord) partialOpenPolys(btopo *brepTopology, canon [][3]float64) (map[int][]int, error) {
	terminals := make([]partialband.Terminal, len(bc.band.Terminals))
	for i, terminal := range bc.band.Terminals {
		terminals[i] = partialband.Terminal{Face: terminal.Face, Loop: terminal.Loop, Seg: terminal.Seg}
	}
	ringV := [][]int{bc.sideV, bc.capV}
	if bc.fillet != nil {
		ringV = bc.fillet.ringV
	}
	return partialband.OpenPolys(partialband.OpenPolyInput{
		Layout: bc.partial, Face: bc.band.Face, Loop: bc.band.Loop,
		Orig: bc.band.Orig.Segments, CapWalk: bc.band.CapWalk, Terminals: terminals,
		RingV: ringV, Canon: canon, Uses: btopo.Uses, FaceUses: btopo.FaceUses,
		Open: btopo.Open, Embed: btopo.Embeds[bc.band.Face], SideZ: bc.sideZ,
	})
}

func (bc *brepBandChord) emitPartialFillet(m *Mesh, faceOfRole func(string) (*Face, error),
	bump func(*Face, float64)) error {
	fr := bc.fillet
	fr.patchEps = map[*Face]float64{}
	r := bc.band.Setback.AxialUpper()
	dphi := proofbound.UpRound(proofbound.UpRound(math.Pi/2*(1+1e-12)) / float64(fr.n))
	sPhi := proofbound.ProductUpper(r, proofbound.UpRound(proofbound.ProductUpper(dphi, dphi)/8))
	delta := bc.face.Delta
	levelDelta := proofbound.AbsSumUpper(bc.band.Setback.DsDelta, bc.sideDelta)
	axial := bc.cbp.CapBandLevel(bc.face.Z0, bc.band.MatSign(bc.face)).Bound
	for p := range bc.partial.PatchSag {
		face, err := faceOfRole(bc.band.PatchRole(p))
		if err != nil {
			return err
		}
		first := len(m.n.Triangles)
		twist := 0.0
		tri := func(a, b, c int) {
			if a != b && b != c && a != c {
				topobuild.AddTriangle(&m.n, [3]int{a, b, c}, faceNode(face))
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
						m.n.Vertices[a], m.n.Vertices[b], m.n.Vertices[A], m.n.Vertices[B]))
				}
			}
		}
		eps := proofbound.AbsSumUpper(sPhi, bc.partial.PatchSag[p], twist)
		bc.finishPatch(m, face, eps, delta, levelDelta, axial, first, len(m.n.Triangles), bump)
	}
	return nil
}

func (bc *brepBandChord) partialMotion(motion, store, round []float64) {
	for _, vi := range bc.sideV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], bc.sideDelta))
	}
	for _, vi := range bc.capV {
		motion[vi] = math.Max(motion[vi], proofbound.AbsSumUpper(store[vi], bc.face.Delta, bc.face.Z0Delta))
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

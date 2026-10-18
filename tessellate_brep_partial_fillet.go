package decad

import (
	"context"

	"github.com/lestrrat-3d/decad/internal/brepfilletmesh"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/partialband"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
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
		fillet: &brepfilletmesh.Rings{Count: tessellation.FilletRingCount(b.Setback.AxialUpper(), chord)}, partial: partial}, nil
}

// placePartial keeps the selected band's boundary vertices in the root mesh adapter.
func (bc *brepBandChord) placePartial(e brepEmbed, addVertex func([3]float64, proofbound.WalkEndBound) int) {
	for _, col := range bc.partial.Columns {
		s, c := col.Side, col.Cap
		bc.sideV = append(bc.sideV, addVertex(e.Canon(s.U, s.V, bc.sideZ), col.SideBound))
		bc.capV = append(bc.capV, addVertex(e.Canon(c.U, c.V, bc.face.Z0), col.CapBound))
	}
}

func (bc *brepBandChord) partialOpenPolys(btopo *brepTopology, canon [][3]float64) (map[int][]int, error) {
	ringV := [][]int{bc.sideV, bc.capV}
	if bc.fillet != nil {
		ringV = bc.fillet.RingV
	}
	return brepfilletmesh.OpenPolys(bc.partial, bc.band, btopo, canon, bc.sideZ, ringV)
}

func (bc *brepBandChord) emitPartialFillet(m *Mesh,
	faceOfRole func(string) (*Face, error), bump func(*topo.Face, float64)) error {
	nodeOfRole := func(role string) (*topo.Face, error) {
		face, err := faceOfRole(role)
		if err != nil {
			return nil, err
		}
		return faceNode(face), nil
	}
	return bc.fillet.EmitPartialWithLevel(bc.filletInput(), bc.partial, &m.n,
		bc.sideDelta, nodeOfRole, bump)
}

func (bc *brepBandChord) partialMotion(motion, store, round []float64) {
	if bc.fillet != nil {
		bc.fillet.PartialMotion(bc.filletInput(), bc.sideDelta, motion, store, round)
		return
	}
	brepfilletmesh.PartialBoundaryMotion(bc.filletInput(), bc.sideDelta, motion, store)
}

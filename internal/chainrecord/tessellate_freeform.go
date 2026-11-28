package chainrecord

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/boundarywalk"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// tessellateCurved chords each walk and shares the held vertices at wall junctions.
func tessellateCurved(ctx context.Context, body *topo.Body, pp Payload, chord float64) (*topo.Mesh, error) {
	if !(chord > 0) {
		return nil, fmt.Errorf("%w: a ribbon needs a positive chord", decaderr.ErrUnsupported)
	}
	budgetChord := chord
	if pp.SectionDelta > 0 {
		budgetChord = freeform.DownRound(freeform.DownRound(chord - pp.SectionDelta))
		if budgetChord <= 0 {
			return nil, fmt.Errorf("%w: requested tolerance leaves no chord budget above the section displacement", decaderr.ErrUnsupported)
		}
	}
	height := proofbound.BoundedSub(pp.Z1Scalar(), pp.Z0Scalar())
	heightUpper := proofbound.AbsSumUpper(math.Abs(height.Value), height.Bound)
	mesh := new(topo.Mesh)
	store := []float64{}
	indices := map[*topo.Vertex]int{}
	addHeld := func(v *topo.Vertex) int {
		if i, ok := indices[v]; ok {
			return i
		}
		i := len(mesh.Vertices)
		mesh.Vertices = append(mesh.Vertices, v.Position)
		store = append(store, v.Bound.Mag())
		indices[v] = i
		return i
	}
	faces := map[string]*topo.Face{}
	for _, face := range topobuild.Faces(body) {
		for _, origin := range face.Origins {
			if origin.Producer == body.Origin.Producer {
				faces[origin.Role] = face
			}
		}
	}
	work := freeform.NewFreeformWork()
	budget := proofbound.NewWorkBudget(ctx)
	view := pp.Prism()
	trims := map[*topo.Face]float64{}
	wallSlack := 0.0
	walkCount := 0
	for ci, chain := range pp.Chains {
		raw := make([]survey2d.SideWalk, len(chain.Segments))
		for i, seg := range chain.Segments {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			walk, err := boundarywalk.WalkOf(seg, work)
			if err != nil {
				return nil, err
			}
			raw[i] = survey2d.SideWalk{SegmentWalk: walk, Segs: []int{i}}
		}
		walks, err := boundarywalk.CoalesceChainWalksContext(ctx, raw)
		if err != nil {
			return nil, err
		}
		walkCount += len(walks)
		for _, walk := range walks {
			if err := budget.Step(); err != nil {
				return nil, err
			}
			face := faces[fmt.Sprintf("side(%d,%d)", ci, walk.Segs[0])]
			if face == nil || len(face.Loops) != 1 || len(face.Loops[0].CoEdges) != 4 || walk.Closed {
				return nil, fmt.Errorf("%w: ribbon walk does not match a four-edge wall", decaderr.ErrUnsupported)
			}
			samples, err := tessellation.SampleLoop([]survey2d.SideWalk{walk}, chain.Segments, budgetChord/2,
				heightUpper, work, budget, func(survey2d.SideWalk) (*topo.Face, error) { return face, nil }, stationbound.ChordStationBound)
			if err != nil {
				return nil, err
			}
			if len(samples.Samples) == 0 {
				return nil, fmt.Errorf("%w: empty ribbon stations", decaderr.ErrUnsupported)
			}
			co := face.Loops[0].CoEdges
			corners := [4]*topo.Vertex{topobuild.CoEdgeStart(co[0]), topobuild.CoEdgeStart(co[1]),
				topobuild.CoEdgeStart(co[2]), topobuild.CoEdgeStart(co[3])}
			lo := []int{addHeld(corners[0])}
			hi := []int{addHeld(corners[3])}
			for i := 1; i < len(samples.Samples); i++ {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				p := samples.Samples[i]
				plane := proofbound.WalkEndBoundAllow(samples.BoundOf[i])
				for j, z := range []float64{pp.Z0, pp.Z1} {
					v, lift := view.LiftedVertex(p.U, p.V, z)
					index := len(mesh.Vertices)
					mesh.Vertices = append(mesh.Vertices, v)
					axial := pp.Z0Delta
					if j == 1 {
						axial = pp.Z1Delta
					}
					store = append(store, proofbound.AbsSumUpper(plane, lift, axial))
					if j == 0 {
						lo = append(lo, index)
					} else {
						hi = append(hi, index)
					}
				}
			}
			lo = append(lo, addHeld(corners[1]))
			hi = append(hi, addHeld(corners[2]))
			for i := range len(samples.Samples) {
				t1, t2 := [3]int{lo[i], lo[i+1], hi[i+1]}, [3]int{lo[i], hi[i+1], hi[i]}
				if pp.Xform.IsReflection() {
					t1[1], t1[2] = t1[2], t1[1]
					t2[1], t2[2] = t2[2], t2[1]
				}
				topobuild.AddTriangle(mesh, t1, face)
				topobuild.AddTriangle(mesh, t2, face)
			}
			trims[face] = math.Max(trims[face], samples.MaxSag)
			wallSlack = proofbound.AbsSumUpper(wallSlack, samples.WallSlack)
		}
	}
	var err error
	mesh.CoordBound, err = tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	bounds, err := tessellation.FaceBounds(mesh.Triangles, mesh.Source, trims, nil, store, pp.SectionDelta)
	if err != nil {
		return nil, err
	}
	for face, bound := range bounds {
		topobuild.SetFaceBound(mesh, face, bound)
	}
	sectionArea := proofbound.ProductUpper(proofbound.SectionDisplacementLength(pp.SectionDelta, walkCount), heightUpper)
	mesh.AreaSlack = proofbound.AbsSumUpper(wallSlack, sectionArea, tessellation.StoreAreaAllow(mesh.Vertices, mesh.Triangles, store))
	if err := topobuild.RequireSheetMesh(ctx, body, mesh); err != nil {
		return nil, err
	}
	return mesh, nil
}

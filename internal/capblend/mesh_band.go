package capblend

import (
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// EmitBand writes the band of lm on the start (or end) cap into mesh: the
// patch cells between the side ring at the band's side level and the cap
// contour ring, with their departure terms. The cap-blend mesh build and the
// brep tessellator's route L bands (internal/brepbandmesh) both call it.
// faceOfRole resolves a patch role to its face, geomOfRole states each
// patch's geometry, and bump raises a face's trim bound. roleOf, when non-nil,
// names patch p's role in place of chamferCap(cap,loop,p): a route L band's
// patches carry chamferLoop(f,l,p) (modify-general BG2).
func EmitBand(budget *proofbound.WorkBudget, mesh *topo.Mesh, cbp Payload, lm *LoopMesh, start bool,
	faceOfRole func(string) (*topo.Face, error), geomOfRole map[string]capband.Patch,
	bump func(*topo.Face, float64), roleOf func(int) string) error {
	matSign := 1.0
	capZ := cbp.Z0
	sideV, capV := lm.SideLo, lm.CapLoV
	if !start {
		matSign, capZ = -1, cbp.Z1
		sideV, capV = lm.SideHi, lm.CapHiV
	}
	setback := cbp.SetbackAt(matSign)
	sideZ := capZ + matSign*setback.Ds
	delta, ok := cbp.BandDelta[BandKey{Loop: lm.LoopIndex, Start: start}]
	if !ok {
		return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, decaderr.ErrDegenerate, lm.LoopIndex)
	}
	levelDelta := proofbound.AbsSumUpper(setback.DsDelta, proofarith.AddRoundError(capZ, matSign*setback.Ds, sideZ))
	axial := cbp.CapBandLevel(capZ, matSign).Bound
	capName := CapNameOf(matSign)
	patchFace := func(p int) (*topo.Face, tessellation.CapBlendBandPatch, error) {
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, lm.LoopIndex, p)
		if cbp.Draft {
			// A draft band has no apex patch, so patch p is walk p, and its
			// face carries the prism's own wall role (draftPayload.Patches).
			role = fmt.Sprintf("side(%d,%d)", lm.LoopIndex, lm.Walks[p].Segs[0])
		}
		if roleOf != nil {
			role = roleOf(p)
		}
		f, err := faceOfRole(role)
		if err != nil {
			return nil, tessellation.CapBlendBandPatch{}, err
		}
		g, ok := geomOfRole[role]
		if !ok {
			return nil, tessellation.CapBlendBandPatch{}, fmt.Errorf(`%w: the payload states no geometry for patch role %q`, decaderr.ErrDegenerate, role)
		}
		return f, tessellation.CapBlendBandPatch{
			Circular: g.Circular, SideRadius: g.SideRadius, CapRadius: g.CapRadius,
			Skew: PatchWindowSkew(g), AreaAllow: capband.DisplacementAreaAllow(g),
		}, nil
	}
	return tessellation.EmitCapBlendBand[*topo.Face](budget, bandWriter{m: mesh, bump: bump},
		tessellation.CapBlendBandInput{
			Loop: lm.LoopIndex, Start: start, Walks: lm.Walks, Joins: SampleJoins(lm.Joins),
			Count: lm.Count, ArcCount: lm.ArcCount,
			SideStart: lm.SideStart, CapWallStart: lm.CapWallStart, CapArcStart: lm.CapArcStart,
			SideV: sideV, CapV: capV,
			SideSag: lm.SideSag, CapSag: lm.CapSag, CapRadius: lm.CapRadius,
			ArcSag: lm.ArcSag, LocusGap: lm.LocusGap,
			D: setback.Dc, Delta: delta, LevelDelta: levelDelta, Axial: axial,
			Amounts: cbp.WalkAmounts(lm.LoopIndex, lm.Walks, setback.Dc),
		}, patchFace)
}

// bandWriter is EmitBand's mesh sink for tessellation.EmitCapBlendBand.
type bandWriter struct {
	m    *topo.Mesh
	bump func(*topo.Face, float64)
}

func (w bandWriter) AddTriangle(tri [3]int, face *topo.Face) { topobuild.AddTriangle(w.m, tri, face) }
func (w bandWriter) Vertices() []r3.Vec                      { return w.m.Vertices }
func (w bandWriter) Triangles() [][3]int                     { return w.m.Triangles }
func (w bandWriter) Bump(face *topo.Face, delta float64)     { w.bump(face, delta) }
func (w bandWriter) AddAreaSlack(patch, facets float64) {
	w.m.AreaSlack = proofbound.AbsSumUpper(w.m.AreaSlack, patch, facets)
}

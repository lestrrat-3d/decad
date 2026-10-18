package decad

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/capband"
	"github.com/lestrrat-3d/decad/internal/capblend"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
)

// tessellateCapBlend adapts the node build to public handles and keeps the
// shared BRep band emitter and Loft occupied-volume proof in root.
func tessellateCapBlend(ctx context.Context, b *Body, cbp capBlendPayload, chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	mesh := bornMesh(new(Mesh))
	emit := func(budget *proofbound.WorkBudget, lm *capblend.LoopMesh, start bool,
		faceOfRole func(string) (*topo.Face, error), geomOfRole map[string]capband.Patch,
		bump func(*topo.Face, float64)) error {
		roleFace := func(role string) (*Face, error) {
			face, err := faceOfRole(role)
			return faceHandle(face), err
		}
		bumpFace := func(face *Face, delta float64) { bump(faceNode(face), delta) }
		return emitCapBand(budget, mesh, cbp, lm, start, roleFace, geomOfRole, bumpFace, nil)
	}
	lms, motion, err := capblend.TessellateMesh(ctx, &b.n, cbp, chord, verify >= VerifyAll, &mesh.n, emit)
	if err != nil {
		return nil, err
	}
	if verify >= VerifyAll && cbp.LoftSource != nil {
		if err := proveLoftCapBandVolume(ctx, mesh, b, cbp, lms, motion); err != nil {
			return nil, err
		}
	}
	return mesh, nil
}

// emitCapBand maps built patch roles to the numeric band emitter. roleOf, when
// non-nil, names patch p's role in place of chamferCap(cap,loop,p): a route L
// band's patches carry chamferLoop(f,l,p) (modify-general BG2).
func emitCapBand(budget *proofbound.WorkBudget, m *Mesh, cbp capBlendPayload, lm *capblend.LoopMesh, start bool, faceOfRole func(string) (*Face, error), geomOfRole map[string]capPatchGeom, bump func(*Face, float64), roleOf func(int) string) error {
	matSign := 1.0
	capZ := cbp.Z0
	sideV, capV := lm.SideLo, lm.CapLoV
	if !start {
		matSign, capZ = -1, cbp.Z1
		sideV, capV = lm.SideHi, lm.CapHiV
	}
	setback := cbp.SetbackAt(matSign)
	sideZ := capZ + matSign*setback.Ds
	delta, ok := cbp.BandDelta[capBandKey{Loop: lm.LoopIndex, Start: start}]
	if !ok {
		return fmt.Errorf(`%w: the payload states no contour displacement for the chamfer band on loop %d`, ErrDegenerate, lm.LoopIndex)
	}
	levelDelta := proofbound.AbsSumUpper(setback.DsDelta, proofarith.AddRoundError(capZ, matSign*setback.Ds, sideZ))
	axial := cbp.CapBandLevel(capZ, matSign).Bound
	capName := capblend.CapNameOf(matSign)
	patchFace := func(p int) (*Face, tessellation.CapBlendBandPatch, error) {
		role := fmt.Sprintf("chamferCap(%s,%d,%d)", capName, lm.LoopIndex, p)
		if cbp.Draft {
			// A draft band has no apex patch, so patch p is walk p, and its
			// face carries the prism's own wall role (draftPayload.Patches).
			role = prismWallRole(lm.LoopIndex, lm.Walks[p].Segs[0])
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
			return nil, tessellation.CapBlendBandPatch{}, fmt.Errorf(`%w: the payload states no geometry for patch role %q`, ErrDegenerate, role)
		}
		return f, tessellation.CapBlendBandPatch{
			Circular: g.Circular, SideRadius: g.SideRadius, CapRadius: g.CapRadius,
			Skew: capblend.PatchWindowSkew(g), AreaAllow: capband.DisplacementAreaAllow(g),
		}, nil
	}
	return tessellation.EmitCapBlendBand[*Face](budget, capBlendBandWriter{m: m, bump: bump},
		tessellation.CapBlendBandInput{
			Loop: lm.LoopIndex, Start: start, Walks: lm.Walks, Joins: capblend.SampleJoins(lm.Joins),
			Count: lm.Count, ArcCount: lm.ArcCount,
			SideStart: lm.SideStart, CapWallStart: lm.CapWallStart, CapArcStart: lm.CapArcStart,
			SideV: sideV, CapV: capV,
			SideSag: lm.SideSag, CapSag: lm.CapSag, CapRadius: lm.CapRadius,
			ArcSag: lm.ArcSag, LocusGap: lm.LocusGap,
			D: setback.Dc, Delta: delta, LevelDelta: levelDelta, Axial: axial,
			Amounts: cbp.WalkAmounts(lm.LoopIndex, lm.Walks, setback.Dc),
		}, patchFace)
}

type capBlendBandWriter struct {
	m    *Mesh
	bump func(*Face, float64)
}

func (w capBlendBandWriter) AddTriangle(tri [3]int, face *Face) {
	topobuild.AddTriangle(&w.m.n, tri, faceNode(face))
}
func (w capBlendBandWriter) Vertices() []r3.Vec             { return w.m.n.Vertices }
func (w capBlendBandWriter) Triangles() [][3]int            { return w.m.n.Triangles }
func (w capBlendBandWriter) Bump(face *Face, delta float64) { w.bump(face, delta) }
func (w capBlendBandWriter) AddAreaSlack(patch, facets float64) {
	w.m.n.AreaSlack = proofbound.AbsSumUpper(w.m.n.AreaSlack, patch, facets)
}

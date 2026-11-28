package revolvetess

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveplan"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
)

// TessellateChainArcSweep meshes an off-axis line or circular chain swept
// through one arc. Adjacent source walls share their junction rings.
func TessellateChainArcSweep(ctx context.Context, b *topo.Body, sp chainSweepPayload,
	chord float64, verify Level) (*topo.Mesh, error) {
	if !sp.Arc || len(sp.Spans) != 0 || sp.Revolve.Full {
		return nil, fmt.Errorf("%w: a chain arc Sweep needs one partial arc wall", decaderr.ErrUnsupported)
	}
	if len(sp.Revolve.Chains) == 1 {
		for _, segment := range sp.Revolve.Chains[0].Segments {
			if _, ok := segment.(sectionrecord.ArcSeg); ok {
				return tessellateChainCircular(ctx, b, sp.Revolve, chord, verify, true)
			}
		}
	}
	if len(sp.Revolve.Chains) == 1 && len(sp.Revolve.Chains[0].Segments) > 1 {
		return tessellateChainLines(ctx, b, sp.Revolve, chord, verify, true, false)
	}
	return tessellateChainLinearRevolve(ctx, b, sp.Revolve, chord, verify, false)
}

// SupportsChainRevolve admits straight chains and an axis-clearing circular meridian.
func SupportsChainRevolve(b *topo.Body, chain revolveplan.ChainPayload) bool {
	if len(chain.Chains) == 0 || b.Kind != topo.BodySheet {
		return false
	}
	if len(chain.Chains) > 1 || chain.SectionDelta != 0 {
		for _, c := range chain.Chains {
			if len(c.Segments) == 0 {
				return false
			}
			for _, segment := range c.Segments {
				switch segment.(type) {
				case lineSeg, sectionrecord.ArcSeg, sectionrecord.CircleSeg:
				default:
					return false
				}
			}
		}
		return true
	}
	if len(chain.Chains[0].Segments) == 0 {
		return false
	}
	hasArc := false
	for _, segment := range chain.Chains[0].Segments {
		switch segment.(type) {
		case sectionrecord.ArcSeg:
			hasArc = true
		case lineSeg:
		default:
			return false
		}
	}
	if hasArc {
		return chain.Ax.RadialLower > 0 || (len(chain.Chains[0].Segments) == 1 && chain.Ax.RadialProof)
	}
	faces := topobuild.Faces(b)
	if len(chain.Chains[0].Segments) > 1 {
		return len(faces) > 0
	}
	if len(faces) != 1 {
		return false
	}
	if chain.Full {
		return len(faces[0].Loops) == 1 || len(faces[0].Loops) == 2
	}
	return len(faces[0].Loops) == 1 && (len(faces[0].Loops[0].CoEdges) == 3 || len(faces[0].Loops[0].CoEdges) == 4)
}

// TessellateChainRevolve chords one wall without closing its free rims.
func TessellateChainRevolve(ctx context.Context, b *topo.Body, chain revolveplan.ChainPayload,
	chord float64, verify Level) (*topo.Mesh, error) {
	if len(chain.Chains) > 1 || chain.SectionDelta != 0 {
		return tessellateTrimmedChains(ctx, b, chain, chord, verify)
	}
	for _, segment := range chain.Chains[0].Segments {
		if _, ok := segment.(sectionrecord.ArcSeg); ok {
			return tessellateChainCircular(ctx, b, chain, chord, verify, false)
		}
	}
	if len(chain.Chains[0].Segments) > 1 {
		return tessellateChainLines(ctx, b, chain, chord, verify, false, true)
	}
	return tessellateChainLinearRevolve(ctx, b, chain, chord, verify, true)
}

func tessellateChainLinearRevolve(ctx context.Context, b *topo.Body, chain revolveplan.ChainPayload,
	chord float64, verify Level, allowPole bool) (*topo.Mesh, error) {
	refuse := func(detail string) (*topo.Mesh, error) {
		return nil, fmt.Errorf(`%w: %s`, decaderr.ErrUnsupported, detail)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if len(chain.Chains) != 1 || len(chain.Chains[0].Segments) != 1 ||
		chain.SectionDelta != 0 || b.Kind != topo.BodySheet || len(topobuild.Faces(b)) != 1 {
		return refuse("a chain revolve mesh needs one straight source wall")
	}
	if _, ok := chain.Chains[0].Segments[0].(lineSeg); !ok {
		return refuse("the chain revolve meridian is not a line")
	}
	rp := chain.Revolve()
	resolved, err := revolveaxis.ResolveChain(ctx, chain.Chains[0],
		freeform.NewFreeformWork(), "chain arc Sweep tessellation", rp.ChargedWalk, rp.Ax.SnapTol)
	if err != nil {
		return nil, err
	}
	if len(resolved.Walks) != 1 || len(resolved.Plane) != 1 ||
		resolved.Walks[0].StartV < 0 || resolved.Walks[0].EndV < 0 ||
		(resolved.Walks[0].StartV == 0 && resolved.Walks[0].EndV == 0) ||
		(!allowPole && (resolved.Walks[0].StartV == 0 || resolved.Walks[0].EndV == 0)) {
		return refuse("the chain arc Sweep needs one line strictly clear of the axis")
	}
	w, plane := resolved.Walks[0], resolved.Plane[0]
	lift := rp.Lift()
	trueStartZ, trueStartR, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.StartU, plane.StartV, plane.StartBound)
	if !ok || (w.StartV > 0 && trueStartR.Lo.Sign() <= 0) ||
		(w.StartV == 0 && (trueStartR.Lo.Sign() != 0 || trueStartR.Hi.Sign() != 0)) {
		return refuse("the chain arc Sweep start is not proven clear of the denoted axis")
	}
	trueEndZ, trueEndR, ok := revolveproof.ChainArcSourcePoint(rp.Ax, plane.EndU, plane.EndV, plane.EndBound)
	if !ok || (w.EndV > 0 && trueEndR.Lo.Sign() <= 0) ||
		(w.EndV == 0 && (trueEndR.Lo.Sign() != 0 || trueEndR.Hi.Sign() != 0)) {
		return refuse("the chain arc Sweep end is not proven clear of the denoted axis")
	}
	firstZ, firstR, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV,
		lift.DU, lift.DV, plane.StartU, plane.StartV, plane.StartBound)
	if !ok || (w.StartV > 0 && firstR.Lo.Sign() <= 0) ||
		(w.StartV == 0 && (firstR.Lo.Sign() != 0 || firstR.Hi.Sign() != 0)) {
		return refuse("the chain arc Sweep start has no positive radial enclosure")
	}
	lastZ, lastR, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV,
		lift.DU, lift.DV, plane.EndU, plane.EndV, plane.EndBound)
	if !ok || (w.EndV > 0 && lastR.Lo.Sign() <= 0) ||
		(w.EndV == 0 && (lastR.Lo.Sign() != 0 || lastR.Hi.Sign() != 0)) {
		return refuse("the chain arc Sweep end has no positive radial enclosure")
	}
	samples := [][]revolvemesh.RevMeridian{{
		{Z: w.StartU, Rho: w.StartV, ZIv: firstZ, RhoIv: firstR, OnAxis: w.StartV == 0},
		{Z: w.EndU, Rho: w.EndV, ZIv: lastZ, RhoIv: lastR, OnAxis: w.EndV == 0},
	}}
	if samples[0][0].Z == samples[0][1].Z && samples[0][0].Rho == samples[0][1].Rho {
		return refuse("the chain arc Sweep's meridian collapses")
	}
	angle0, ok0 := rp.Den.Phi0.Enclosure()
	angle1, ok1 := rp.Den.Phi1.Enclosure()
	if !ok0 || !ok1 {
		return refuse("the chain arc Sweep has no enclosed angular interval")
	}
	span := proofbound.IntervalSub(angle1, angle0)
	if span.Lo.Sign() <= 0 {
		return refuse("the chain arc Sweep needs a positive enclosed angle")
	}
	sweep := proofbound.RatFloatUp(span.Hi)
	radiusUpper := trueStartR.Hi
	if trueEndR.Hi.Cmp(radiusUpper) > 0 {
		radiusUpper = trueEndR.Hi
	}
	radius := proofbound.RatFloatUp(radiusUpper)
	if proofbound.IsNonFinite(sweep) || proofbound.IsNonFinite(radius) {
		return refuse("the chain arc Sweep has no finite chord envelope")
	}
	ideal, ok := revolvemesh.IdealBasis(lift.Frame, lift.AU, lift.AV, lift.DU, lift.DV)
	if !ok {
		return refuse("the chain arc Sweep has no enclosed axis basis")
	}
	face := topobuild.Faces(b)[0]
	for n := 8; n <= maxChainSweepArcCells; n *= 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sag := tessellation.ChordSagitta(radius, sweep, n)
		if proofbound.IsNonFinite(sag) || sag >= chord {
			continue
		}
		angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{
			Phi0: rp.Phi0, Phi1: rp.Phi1, Full: rp.Full, Den: rp.Den,
		}, n)
		if err != nil {
			return nil, err
		}
		budget := proofbound.NewWorkBudget(ctx)
		verts, deltaC, deltaR, err := revolvemesh.EmitRings(samples, angular,
			lift.Basis(), ideal, rp.Xform, rp.Full, budget)
		if err != nil {
			return nil, err
		}
		coord := proofbound.AbsSumUpper(deltaC, deltaR)
		vertexGap := 0.0
		for j, source := range [2]struct {
			u, v  float64
			bound proofbound.WalkEndBound
		}{
			{plane.StartU, plane.StartV, plane.StartBound},
			{plane.EndU, plane.EndV, plane.EndBound},
		} {
			for l := range angular.Samples {
				if err := budget.Step(); err != nil {
					return nil, err
				}
				gap := lift.SweptPointGap(rp.AxisBound(), rp.Xform,
					source.u, source.v, source.bound, angular.SinIv[l], angular.CosIv[l],
					verts[samples[0][j].At(l)])
				vertexGap = math.Max(vertexGap, gap)
			}
		}
		bound := proofbound.UpRound(proofbound.AbsSumUpper(sag, vertexGap))
		if proofbound.IsNonFinite(bound) || bound > chord {
			continue
		}
		mesh := &topo.Mesh{Vertices: verts, CoordBound: math.Max(coord, vertexGap)}
		revolvemesh.EmitCellTriangles(samples[0][0], samples[0][1], n, func(tri [3]int) {
			topobuild.AddTriangle(mesh, tri, face)
		})
		if rp.Reflected() {
			for i := range mesh.Triangles {
				mesh.Triangles[i][1], mesh.Triangles[i][2] = mesh.Triangles[i][2], mesh.Triangles[i][1]
			}
		}
		topobuild.SetFaceBound(mesh, face, bound)
		if err := topobuild.RequireSheetMesh(ctx, b, mesh); err != nil {
			return nil, err
		}
		audit, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.Vertices, mesh.Triangles,
			mesh.CoordBound)
		if err != nil {
			return nil, topobuild.LiftAuditError(err)
		}
		if verify >= VerifyBoundary {
			if err := revolvemesh.RevolveContactAudit(budget, audit, mesh.Triangles,
				mesh.CoordBound); err != nil {
				return nil, topobuild.LiftAuditError(err)
			}
		}
		if verify >= VerifyAll {
			axisAllow, err := revolveproof.ChainArcAreaDensityAllow(trueStartZ, trueStartR, trueEndZ,
				trueEndR, w.StartU, w.StartV, w.EndU, w.EndV, span, angular.Step, n)
			if err != nil {
				return nil, err
			}
			cell, err := revolveproof.CellSlack(ideal, angular, samples[0][0], samples[0][1], coord)
			if err != nil {
				return nil, err
			}
			mesh.AreaSlack = proofbound.AbsSumUpper(
				proofbound.ProductUpper(float64(n), proofbound.AbsSumUpper(cell, axisAllow)),
				revolvemesh.CoordinateAreaAllow(mesh.Vertices, mesh.Triangles, coord))
			if proofbound.IsNonFinite(mesh.AreaSlack) {
				return refuse("the chain arc Sweep has no finite area slack")
			}
		}
		return mesh, nil
	}
	return refuse("the chain arc Sweep needs more than 4096 chord cells at this tolerance")
}

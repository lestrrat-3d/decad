package decad

import (
	"context"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/freeform"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolveaxis"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/revolveproof"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
)

// tessellateChainArcSweep restates one straight, off-axis source chain swept
// through one arc. The two endpoint rings are the complete open meridian;
// their angular cells share vertices by index and leave exactly four rims.
// Other chain shapes need their own meridian station and clearance proof.
func tessellateChainArcSweep(ctx context.Context, b *Body, sp chainSweepPayload,
	chord float64, verify Verification) (*Mesh, error) {
	refuse := func(detail string) (*Mesh, error) {
		return nil, fmt.Errorf(`%w: %s`, ErrUnsupported, detail)
	}
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	if !sp.arc || len(sp.spans) != 0 || len(sp.revolve.chains) != 1 ||
		len(sp.revolve.chains[0].Segments) != 1 || sp.revolve.full ||
		sp.revolve.sectionDelta != 0 || b.Kind() != BodySheet || len(b.Faces()) != 1 {
		return refuse("a chain arc Sweep mesh needs one straight source and one partial arc wall")
	}
	if _, ok := sp.revolve.chains[0].Segments[0].(lineSeg); !ok {
		return refuse("the chain arc Sweep meridian is not a line")
	}
	rp := sp.revolve.revolve()
	resolved, err := revolveaxis.ResolveChain(ctx, sp.revolve.chains[0],
		freeform.NewFreeformWork(), "chain arc Sweep tessellation", rp.chargedWalk, rp.ax.SnapTol)
	if err != nil {
		return nil, err
	}
	if len(resolved.Walks) != 1 || len(resolved.Plane) != 1 ||
		resolved.Walks[0].StartV <= 0 || resolved.Walks[0].EndV <= 0 {
		return refuse("the chain arc Sweep needs one line strictly clear of the axis")
	}
	w, plane := resolved.Walks[0], resolved.Plane[0]
	lift := rp.lift()
	trueStartZ, trueStartR, ok := revolveproof.ChainArcSourcePoint(rp.ax, plane.StartU, plane.StartV, plane.StartBound)
	if !ok || trueStartR.Lo.Sign() <= 0 {
		return refuse("the chain arc Sweep start is not proven clear of the denoted axis")
	}
	trueEndZ, trueEndR, ok := revolveproof.ChainArcSourcePoint(rp.ax, plane.EndU, plane.EndV, plane.EndBound)
	if !ok || trueEndR.Lo.Sign() <= 0 {
		return refuse("the chain arc Sweep end is not proven clear of the denoted axis")
	}
	firstZ, firstR, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV,
		lift.DU, lift.DV, plane.StartU, plane.StartV, plane.StartBound)
	if !ok || firstR.Lo.Sign() <= 0 {
		return refuse("the chain arc Sweep start has no positive radial enclosure")
	}
	lastZ, lastR, ok := revolvemesh.RevolveMeridianEnclosure(lift.AU, lift.AV,
		lift.DU, lift.DV, plane.EndU, plane.EndV, plane.EndBound)
	if !ok || lastR.Lo.Sign() <= 0 {
		return refuse("the chain arc Sweep end has no positive radial enclosure")
	}
	samples := [][]revolvemesh.RevMeridian{{
		{Z: w.StartU, Rho: w.StartV, ZIv: firstZ, RhoIv: firstR},
		{Z: w.EndU, Rho: w.EndV, ZIv: lastZ, RhoIv: lastR},
	}}
	if samples[0][0].Z == samples[0][1].Z && samples[0][0].Rho == samples[0][1].Rho {
		return refuse("the chain arc Sweep's meridian collapses")
	}
	angle0, ok0 := rp.den.Phi0.Enclosure()
	angle1, ok1 := rp.den.Phi1.Enclosure()
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
	face := b.Faces()[0]
	for n := 8; n <= maxChainSweepArcCells; n *= 2 {
		if err := ctx.Err(); err != nil {
			return nil, err
		}
		sag := tessellation.ChordSagitta(radius, sweep, n)
		if proofbound.IsNonFinite(sag) || sag >= chord {
			continue
		}
		angular, err := revolvemesh.AngularSequence(revolvemesh.AngularInput{
			Phi0: rp.phi0, Phi1: rp.phi1, Den: rp.den,
		}, n)
		if err != nil {
			return nil, err
		}
		budget := proofbound.NewWorkBudget(ctx)
		verts, deltaC, deltaR, err := revolvemesh.EmitRings(samples, angular,
			lift.Basis(), ideal, rp.xform, false, budget)
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
				gap := lift.SweptPointGap(rp.axisBound(), rp.xform,
					source.u, source.v, source.bound, angular.SinIv[l], angular.CosIv[l],
					verts[samples[0][j].At(l)])
				vertexGap = math.Max(vertexGap, gap)
			}
		}
		bound := proofbound.UpRound(proofbound.AbsSumUpper(sag, vertexGap))
		if proofbound.IsNonFinite(bound) || bound > chord {
			continue
		}
		mesh := bornMesh(&Mesh{n: topo.Mesh{Vertices: verts, CoordBound: math.Max(coord, vertexGap)}})
		revolvemesh.EmitCellTriangles(samples[0][0], samples[0][1], n, func(tri [3]int) {
			mesh.addTriangle(tri, faceNode(face))
		})
		if rp.reflected() {
			for i := range mesh.n.Triangles {
				mesh.n.Triangles[i][1], mesh.n.Triangles[i][2] = mesh.n.Triangles[i][2], mesh.n.Triangles[i][1]
			}
		}
		mesh.setFaceBound(faceNode(face), bound)
		if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
			return nil, err
		}
		audit, err := revolvemesh.RequireRevolveFacetAreas(budget, mesh.n.Vertices, mesh.n.Triangles,
			mesh.n.CoordBound)
		if err != nil {
			return nil, liftTessellationError(err)
		}
		if verify >= VerifyBoundary {
			if err := revolvemesh.RevolveContactAudit(budget, audit, mesh.n.Triangles,
				mesh.n.CoordBound); err != nil {
				return nil, liftTessellationError(err)
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
			mesh.n.AreaSlack = proofbound.AbsSumUpper(
				proofbound.ProductUpper(float64(n), proofbound.AbsSumUpper(cell, axisAllow)),
				revolvemesh.CoordinateAreaAllow(mesh.n.Vertices, mesh.n.Triangles, coord))
			if proofbound.IsNonFinite(mesh.n.AreaSlack) {
				return refuse("the chain arc Sweep has no finite area slack")
			}
		}
		return mesh, nil
	}
	return refuse("the chain arc Sweep needs more than 4096 chord cells at this tolerance")
}

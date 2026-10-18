package decad

import (
	"context"
	"fmt"
	"math/big"

	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/revolvemesh"
	"github.com/lestrrat-3d/decad/internal/stationbound"
	"github.com/lestrrat-3d/decad/internal/tessellation"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
)

// The line/half-circle chain sweep is a planar rectangle joined to a half
// annulus. Its one recorded line supplies the two profile stations at every
// angular section. The join reuses the straight wall's own two vertices, and
// the far rim reuses the arc wall's vertices. No coordinate-based weld occurs.
const maxChainSweepArcCells = 4096

func tessellateChainSweep(ctx context.Context, b *Body, sp chainSweepPayload,
	chord float64, verify Verification) (*Mesh, error) {
	if err := ctx.Err(); err != nil {
		return nil, err
	}
	w, height, radius, straight, curved, err := chainSweepMeshClass(b, sp)
	if err != nil {
		return nil, err
	}
	upperPi := proofbound.RatFloatUp(proofbound.PiUpper)
	n := 8
	for n <= maxChainSweepArcCells && tessellation.ChordSagitta(radius, upperPi, n) > chord/2 {
		n *= 2
	}
	if n > maxChainSweepArcCells {
		return nil, fmt.Errorf(`%w: the chain sweep arc needs more than %d chord cells at this tolerance`,
			ErrUnsupported, maxChainSweepArcCells)
	}

	lineCo := straight.Loops()[0].CoEdges()
	arcCo := curved.Loops()[0].CoEdges()
	if len(lineCo) != 4 || len(arcCo) != 4 ||
		arcCo[0].Start() != lineCo[3].Start() || arcCo[1].Start() != lineCo[2].Start() {
		return nil, fmt.Errorf(`%w: the chain sweep walls do not share their two indexed join vertices`, ErrUnsupported)
	}
	// The straight wall's coedge order is source-start at its near rim,
	// source-end at its near rim, source-end at the join, source-start at
	// the join. The arc wall walks the same source indices at its start.
	mesh := bornMesh(&Mesh{})
	store := make([]float64, 0, 2*n+4)
	for i, nominal := range []r3.Vec{{}, {X: w}, {X: w, Z: height}, {Z: height}} {
		v := lineCo[i].Start()
		mesh.n.Vertices = append(mesh.n.Vertices, v.n.Position)
		store = append(store, tessellation.ChainSweepVertexGap(v.n.Position, v.n.Bound.Mag(), nominal))
	}
	topobuild.AddTriangle(&mesh.n, [3]int{0, 1, 2}, faceNode(straight))
	topobuild.AddTriangle(&mesh.n, [3]int{0, 2, 3}, faceNode(straight))
	outer, inner := 3, 2
	budget := proofbound.NewWorkBudget(ctx)
	for j := 1; j <= n; j++ {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		nextOuter, nextInner := len(mesh.n.Vertices), len(mesh.n.Vertices)+1
		for k, radial := range []float64{radius, radius - w} {
			var p r3.Vec
			var gap float64
			if j == n {
				v := arcCo[3-k].Start()
				p = v.n.Position
				x := new(big.Rat).Mul(proofarith.FloatRat(radius), big.NewRat(2, 1))
				if k == 1 {
					x.Sub(x, proofarith.FloatRat(w))
				}
				gap = tessellation.ChainSweepVertexGapRat(v.n.Position, v.n.Bound.Mag(),
					x, new(big.Rat), proofarith.FloatRat(height))
			} else {
				p, gap, err = stationbound.ChainSweepArcStation(height, radius, radial, j, n)
				if err != nil {
					return nil, err
				}
			}
			mesh.n.Vertices = append(mesh.n.Vertices, p)
			store = append(store, gap)
		}
		topobuild.AddTriangle(&mesh.n, [3]int{outer, inner, nextInner}, faceNode(curved))
		topobuild.AddTriangle(&mesh.n, [3]int{outer, nextInner, nextOuter}, faceNode(curved))
		outer, inner = nextOuter, nextInner
	}
	if err := budget.Err(); err != nil {
		return nil, err
	}
	coord, err := tessellation.StoreMax(store)
	if err != nil {
		return nil, err
	}
	mesh.n.CoordBound = coord
	straightBound := max(store[0], store[1], store[2], store[3])
	topobuild.SetFaceBound(&mesh.n, faceNode(straight), proofbound.UpRound(straightBound))
	arcBound := proofbound.AbsSumUpper(tessellation.ChordSagitta(radius, upperPi, n), coord)
	if proofbound.IsNonFinite(arcBound) || arcBound > chord {
		return nil, fmt.Errorf(`%w: the chain sweep arc's chord and coordinate bound exceeds the requested tolerance`,
			ErrUnsupported)
	}
	topobuild.SetFaceBound(&mesh.n, faceNode(curved), arcBound)

	// A per-triangle exact area bracket checks positive area at every level.
	// At VerifyAll, the two live faces' certified area intervals bound the
	// difference from their attributed held triangle sums.
	areas := map[*topo.Face]proofbound.RatInterval{}
	for i, tri := range mesh.n.Triangles {
		if err := budget.Step(); err != nil {
			return nil, err
		}
		lo, hi, ok := tessellation.ChainSweepTriangleArea(mesh.n.Vertices, tri)
		if !ok {
			return nil, fmt.Errorf(`%w: chain sweep facet %d has no positive area`, ErrUnsupported, i)
		}
		pair := areas[mesh.n.Source[i]]
		if pair.Lo == nil {
			pair.Lo, pair.Hi = new(big.Rat), new(big.Rat)
		}
		pair.Lo.Add(pair.Lo, proofarith.FloatRat(lo))
		pair.Hi.Add(pair.Hi, proofarith.FloatRat(hi))
		areas[mesh.n.Source[i]] = pair
	}
	if verify >= VerifyBoundary {
		data, auditErr := revolvemesh.RequireRevolveFacetAreas(budget, mesh.n.Vertices, mesh.n.Triangles, coord)
		if auditErr != nil {
			return nil, topobuild.LiftAuditError(auditErr)
		}
		if auditErr = revolvemesh.RevolveContactAudit(budget, data, mesh.n.Triangles, coord); auditErr != nil {
			return nil, topobuild.LiftAuditError(auditErr)
		}
	}
	if verify >= VerifyAll {
		for _, f := range []*Face{straight, curved} {
			slack, proofErr := tessellation.ChainSweepFaceAreaSlack(
				f.n.Area, f.n.AreaBound, areas[faceNode(f)])
			if proofErr != nil {
				return nil, proofErr
			}
			mesh.n.AreaSlack = proofbound.AbsSumUpper(mesh.n.AreaSlack, slack)
		}
		if proofbound.IsNonFinite(mesh.n.AreaSlack) {
			return nil, fmt.Errorf(`%w: the chain sweep mesh has no finite area bound`, ErrUnsupported)
		}
	}
	if err := requireMeshAudit(ctx, true, b, mesh); err != nil {
		return nil, err
	}
	// A sheet has no occupied volume, so symDiffOK stays false.
	return mesh, nil
}

// chainSweepMeshClass admits exactly the producer's cardinal one-line class.
// Other line and arc orientations need a separate certified station map.
func chainSweepMeshClass(b *Body, sp chainSweepPayload) (float64, float64, float64, *Face, *Face, error) {
	refuse := func() (float64, float64, float64, *Face, *Face, error) {
		return 0, 0, 0, nil, nil,
			fmt.Errorf(`%w: this chain sweep mesh requires one cardinal line and one positive half-circle`, ErrUnsupported)
	}
	if b.Kind() != BodySheet || len(sp.Spans) != 2 || sp.Spans[0].Arc || !sp.Spans[1].Arc ||
		sp.Spans[1].ReverseArcRims || sp.Path == nil || len(sp.Path.Segments) != 2 ||
		sp.Spans[0].Chain.Xform != r3.Identity() ||
		sp.Spans[1].Revolve.Xform != r3.Identity() || len(sp.Spans[0].Chain.Chains) != 1 ||
		len(sp.Spans[1].Revolve.Chains) != 1 {
		return refuse()
	}
	chain := sp.Spans[0].Chain.Chains[0]
	if len(chain.Segments) != 1 || len(sp.Spans[1].Revolve.Chains[0].Segments) != 1 {
		return refuse()
	}
	seg, ok := chain.Segments[0].(lineSeg)
	if !ok || seg.Start.U != 0 || seg.Start.V != 0 || seg.End.V != 0 || seg.End.U <= 0 ||
		sp.Spans[0].Chain.Frame.Origin() != (r3.Vec{}) ||
		sp.Spans[0].Chain.Frame.U() != (r3.Vec{X: 1}) ||
		sp.Spans[0].Chain.Frame.V() != (r3.Vec{Y: 1}) {
		return refuse()
	}
	w := seg.End.U
	line, ok := sp.Path.Segments[0], !sp.Path.Segments[0].Arc
	if !ok || sp.Path.Start != (r3.Vec{}) || line.End.X != 0 || line.End.Y != 0 || line.End.Z <= 0 {
		return refuse()
	}
	height := line.End.Z
	arc, ok := sp.Path.Segments[1], sp.Path.Segments[1].Arc
	if !ok || arc.Through.Y != 0 || arc.End.Y != 0 || arc.End.Z != height {
		return refuse()
	}
	radius := arc.Through.X
	if !proofbound.FiniteVec(arc.Through) || !proofbound.FiniteVec(arc.End) || radius <= w ||
		proofarith.FloatRat(arc.Through.Z).Cmp(new(big.Rat).Add(proofarith.FloatRat(height), proofarith.FloatRat(radius))) != 0 ||
		proofarith.FloatRat(arc.End.X).Cmp(new(big.Rat).Mul(proofarith.FloatRat(radius), big.NewRat(2, 1))) != 0 ||
		proofarith.FloatRat(radius-w).Cmp(new(big.Rat).Sub(proofarith.FloatRat(radius), proofarith.FloatRat(w))) != 0 {
		return refuse()
	}
	if sp.Spans[0].Chain.Z0 != 0 || sp.Spans[0].Chain.Z1 != height ||
		sp.Spans[0].Chain.SectionDelta != 0 || sp.Spans[1].Revolve.SectionDelta != 0 ||
		sp.Spans[1].Revolve.Frame.Origin() != (r3.Vec{Z: height}) ||
		sp.Spans[1].Revolve.Frame.U() != (r3.Vec{X: 1}) ||
		sp.Spans[1].Revolve.Frame.V() != (r3.Vec{Y: 1}) ||
		sp.Spans[1].Revolve.Ax.AU != radius || sp.Spans[1].Revolve.Ax.AV != 0 ||
		sp.Spans[1].Revolve.Ax.DU != 0 || sp.Spans[1].Revolve.Ax.DV != 1 ||
		sp.Spans[1].Revolve.Ax.AUBound != 0 || sp.Spans[1].Revolve.Ax.AVBound != 0 ||
		sp.Spans[1].Revolve.Ax.DUBound != 0 || sp.Spans[1].Revolve.Ax.DVBound != 0 {
		return refuse()
	}
	var straight, curved *Face
	for _, f := range b.Faces() {
		for _, origin := range f.Origins() {
			switch origin.Role {
			case "side(0,0,0)":
				straight = f
			case "side(1,0,0)":
				curved = f
			}
		}
	}
	if straight == nil || curved == nil || straight == curved || len(b.Faces()) != 2 ||
		len(straight.Loops()) != 1 || len(curved.Loops()) != 1 {
		return refuse()
	}
	return w, height, radius, straight, curved, nil
}

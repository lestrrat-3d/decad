package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/facetproof"
	"github.com/lestrrat-3d/decad/internal/loftmesh"
	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/prismrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/r3"
)

// OutputBoundary is private, transient and has no occupied-volume or
// measurement claim. The source records must remain live throughout its call.
// HeldAudit, AuditedVerts and AuditedTris are set only when the caller asked
// CertifyOutputBoundary to keep the audit.
type OutputBoundary struct {
	Source       SourcePartition
	Shell        OutputShell
	WorkVisits   uint64
	HeldAudit    *facetproof.MeshAudit
	AuditedVerts []r3.Vec
	AuditedTris  [][3]int
}

// CertifyOutputBoundary builds and audits the held output shell of the prism
// pp and the Loft lp from source, their certified partition. The output phase
// runs under its own wall limit, also charged to operation. keepAudit charges
// the shell size once more, audits a copy of the triangles and keeps the held
// audit, vertices and that copy for a published direct shell. arithmetic is
// the sticky state of the partition that produced source.
func CertifyOutputBoundary(ctx context.Context, operation *proofbound.WorkBudget,
	pp prismrecord.Payload, lp loftrecord.Payload, source SourcePartition,
	limit uint64, keepAudit bool, arithmetic *Arithmetic) (OutputBoundary, bool, error) {
	var no OutputBoundary
	wall := proofbound.NewWallWorkBudgetWithOperation(limit, operation)
	var visits uint64
	budget := &proofbound.WorkBudget{
		StepFn: func() error {
			if err := wall.Step(); err != nil {
				return err
			}
			visits++
			return nil
		},
		ErrFn: wall.Err,
	}
	prismLines, ok, err := PartitionLines(budget, pp.Profile.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	bottom, ok, err := PartitionLines(budget, lp.Profile0.Outer)
	if err != nil || !ok {
		return no, false, err
	}
	top, ok, err := PartitionLines(budget, lp.Profile1.Outer)
	if err != nil || !ok || len(top) != len(bottom) {
		return no, false, err
	}
	shell := NewOutputShell()
	for cap := range 3 {
		ok, err = OutputCap(ctx, budget, pp.Frame, prismLines, bottom, top, source, &shell, cap, arithmetic)
		if err != nil || !ok || !arithmetic.OK {
			return no, false, err
		}
	}
	ok, err = OutputSides(budget, pp.Frame, prismLines, bottom, top, source, &shell, arithmetic)
	if err != nil || !ok || !arithmetic.OK {
		return no, false, err
	}
	ok, err = AuditOutputSource(budget, shell, source, arithmetic)
	if err != nil || !ok || !arithmetic.OK {
		return no, false, err
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if keepAudit {
		if _, err := budget.StepN(uint64(len(shell.Vertices) + len(shell.Triangles))); err != nil {
			return no, false, err
		}
		if err := budget.Err(); err != nil {
			return no, false, err
		}
	}
	verts := make([]r3.Vec, len(shell.Vertices))
	for i, v := range shell.Vertices {
		if err := budget.Step(); err != nil {
			return no, false, err
		}
		verts[i] = v.Held
	}
	var auditedTris [][3]int
	if keepAudit {
		auditedTris = append([][3]int(nil), shell.Triangles...)
	}
	// The exact source labels and orientation are already fixed. Neither audit
	// may reverse a triangle to repair an ownership mistake.
	if err := loftmesh.LoftCrossingAuditPrivateBounded(budget, verts, shell.Triangles); err != nil {
		return no, false, err
	}
	auditTris := shell.Triangles
	if keepAudit {
		auditTris = auditedTris
	}
	audit, audited, err := facetproof.AuditFacetedMeshBounded(ctx, budget, verts, auditTris)
	if err != nil || !audited {
		return no, false, err
	}
	if audit == nil || len(audit.Members) != 1 {
		return no, false, nil
	}
	if err := budget.Err(); err != nil {
		return no, false, err
	}
	if !arithmetic.OK {
		return no, false, nil
	}
	out := OutputBoundary{Source: source, Shell: shell, WorkVisits: visits}
	if keepAudit {
		out.HeldAudit, out.AuditedVerts, out.AuditedTris = audit, verts, auditedTris
	}
	return out, true, nil
}

// OutputSides adds the prism and Loft side walls of source to shell. frame is
// the prism frame; prismLines, bottom and top are the line-only source loops.
func OutputSides(budget *proofbound.WorkBudget, frame r3.Frame,
	prismLines, bottom, top []sectionrecord.LineSeg, source SourcePartition,
	shell *OutputShell, checked ...*Arithmetic) (bool, error) {
	arithmetic := ReplayArithmetic(checked)
	z0, zL, zP := source.CapLevels[0], source.CapLevels[1], source.CapLevels[2]
	var lineCut, carrierCut [2][2]*big.Rat
	for k, seam := range source.Seams {
		for t := range 2 {
			if err := budget.Step(); err != nil {
				return false, err
			}
			var ok bool
			lineCut[t][k], carrierCut[t][k], ok = seam.At(big.NewRat(int64(t), 1))
			if !ok || !arithmetic.Valid(lineCut[t][k]) || !arithmetic.Valid(carrierCut[t][k]) {
				return false, nil
			}
		}
		mid, midCarrier, ok := seam.At(big.NewRat(1, 2))
		if !ok || !arithmetic.Valid(mid) || !arithmetic.Valid(midCarrier) {
			return false, nil
		}
		moving := source.Crossing[k]
		midStart, okStart := outputMovingPoint(bottom[moving].Start, top[moving].Start,
			big.NewRat(1, 2))
		midEnd, okEnd := outputMovingPoint(bottom[moving].End, top[moving].End,
			big.NewRat(1, 2))
		carrierPoint := arithmetic.LinePoint(prismLines[source.Carrier], midCarrier)
		interpolated := arithmetic.Interpolate(midStart, midEnd, mid)
		if !okStart || !okEnd || !arithmetic.SamePoint(interpolated, carrierPoint) || !arithmetic.OK {
			return false, nil
		}
		linear := arithmetic.Quo(arithmetic.Add(carrierCut[0][k], carrierCut[1][k]), big.NewRat(2, 1))
		if arithmetic.Cmp(midCarrier, linear) != 0 || !arithmetic.OK {
			return false, nil
		}
		// CertifyLineSeam establishes unique transverse intersections over the
		// interval. Its carrier numerator is quadratic and denominator linear;
		// agreement at 0, 1/2 and 1 with this line is a polynomial identity.
	}
	for i, line := range prismLines {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if i != source.Carrier {
			for band, levels := range [][2]*big.Rat{{z0, zL}, {zL, zP}} {
				ok, err := OutputLineQuad(budget, frame, shell, line, line,
					big.NewRat(0, 1), big.NewRat(1, 1), big.NewRat(0, 1), big.NewRat(1, 1),
					levels[0], levels[1], OutputFace{Kind: 1, Owner: 0, Segment: i, Level: band})
				if err != nil || !ok {
					return false, err
				}
			}
			continue
		}
		for part, bounds := range [][4]*big.Rat{
			{big.NewRat(0, 1), carrierCut[0][0], big.NewRat(0, 1), carrierCut[1][0]},
			{carrierCut[0][1], big.NewRat(1, 1), carrierCut[1][1], big.NewRat(1, 1)},
		} {
			ok, err := OutputLineQuad(budget, frame, shell, line, line,
				bounds[0], bounds[1], bounds[2], bounds[3], z0, zL,
				OutputFace{Kind: 1, Owner: 0, Segment: i, Level: part})
			if err != nil || !ok {
				return false, err
			}
		}
		breaks := [4]*big.Rat{big.NewRat(0, 1), carrierCut[1][0], carrierCut[1][1], big.NewRat(1, 1)}
		for part := range 3 {
			ok, err := OutputLineQuad(budget, frame, shell, line, line,
				breaks[part], breaks[part+1], breaks[part], breaks[part+1], zL, zP,
				OutputFace{Kind: 1, Owner: 0, Segment: i, Level: part + 2})
			if err != nil || !ok {
				return false, err
			}
		}
	}
	for i := range bottom {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if source.Walls[i] == -1 {
			continue
		}
		if !OutputPlanarWall(bottom[i], top[i], zL) {
			return false, nil
		}
		lo, hi := big.NewRat(0, 1), big.NewRat(1, 1)
		upperLo, upperHi := big.NewRat(0, 1), big.NewRat(1, 1)
		switch source.Walls[i] {
		case 0:
			k := 0
			if source.Crossing[1] == i {
				k = 1
			} else if source.Crossing[0] != i {
				return false, nil
			}
			side := arithmetic.LineSide(prismLines[source.Carrier], bottom[i].Start)
			if !arithmetic.OK || side.Sign() == 0 {
				return false, nil
			}
			if side.Sign() < 0 {
				hi, upperHi = lineCut[0][k], lineCut[1][k]
			} else {
				lo, upperLo = lineCut[0][k], lineCut[1][k]
			}
		case 1:
		default:
			return false, nil
		}
		ok, err := OutputLineQuad(budget, frame, shell, bottom[i], top[i],
			lo, hi, upperLo, upperHi, z0, zL,
			OutputFace{Kind: 1, Owner: 1, Segment: i})
		if err != nil || !ok {
			return false, err
		}
	}
	return arithmetic.OK, nil
}

// OutputLineQuad adds the planar convex quadrilateral between low on [lo0,
// hi0] at height z0 and high on [lo1, hi1] at height z1 to shell as two
// triangles labelled face. It refuses a degenerate, nonplanar or nonconvex
// quadrilateral.
func OutputLineQuad(budget *proofbound.WorkBudget, frame r3.Frame,
	shell *OutputShell, low, high sectionrecord.LineSeg,
	lo0, hi0, lo1, hi1, z0, z1 *big.Rat, face OutputFace) (bool, error) {
	if err := budget.Step(); err != nil {
		return false, err
	}
	arithmetic := &Arithmetic{OK: true}
	if arithmetic.Cmp(lo0, hi0) >= 0 || arithmetic.Cmp(lo1, hi1) >= 0 ||
		arithmetic.Cmp(z0, z1) >= 0 || !arithmetic.OK {
		return false, nil
	}
	params := [4]*big.Rat{lo0, hi0, hi1, lo1}
	lines := [4]sectionrecord.LineSeg{low, low, high, high}
	levels := [4]*big.Rat{z0, z0, z1, z1}
	var quad [4]OutputPoint
	for i := range quad {
		p := arithmetic.LinePoint(lines[i], params[i])
		if !arithmetic.OK {
			return false, nil
		}
		quad[i] = OutputP2(p, levels[i])
	}
	if !outputQuadPlanarConvex(quad) {
		return false, nil
	}
	for _, tri := range [][3]int{{0, 1, 2}, {0, 2, 3}} {
		if err := budget.Step(); err != nil {
			return false, err
		}
		if !shell.Triangle(frame,
			[3]OutputPoint{quad[tri[0]], quad[tri[1]], quad[tri[2]]}, face) {
			return false, nil
		}
	}
	return true, nil
}

func outputHeldExact(v r3.Vec) Vec {
	return Vec{proofarith.FloatRat(v.X), proofarith.FloatRat(v.Y), proofarith.FloatRat(v.Z)}
}

// OutputNormalsAgree reports whether the exact triangle a, b, c and its held
// float triangle have normals with a positive dot product.
func OutputNormalsAgree(a, b, c OutputVertex) bool {
	arithmetic := &Arithmetic{OK: true}
	source := arithmetic.Cross(arithmetic.Diff(b.World, a.World), arithmetic.Diff(c.World, a.World))
	ha, hb, hc := outputHeldExact(a.Held), outputHeldExact(b.Held), outputHeldExact(c.Held)
	held := arithmetic.Cross(arithmetic.Diff(hb, ha), arithmetic.Diff(hc, ha))
	dot := arithmetic.Dot(source, held)
	return arithmetic.OK && dot.Sign() > 0
}

func outputMovingPoint(low, high sectionrecord.Point2, t *big.Rat) (Point, bool) {
	a, okA := PointOf(low)
	b, okB := PointOf(high)
	if !okA || !okB {
		return Point{}, false
	}
	arithmetic := &Arithmetic{OK: true}
	point := Point{
		U: arithmetic.Add(a.U, arithmetic.Mul(t, arithmetic.Sub(b.U, a.U))),
		V: arithmetic.Add(a.V, arithmetic.Mul(t, arithmetic.Sub(b.V, a.V))),
	}
	return point, arithmetic.OK
}

func outputDelta(arithmetic *Arithmetic, a, b OutputPoint) Vec {
	return Vec{arithmetic.Sub(b.U, a.U), arithmetic.Sub(b.V, a.V), arithmetic.Sub(b.Z, a.Z)}
}

// outputDet3 returns the triple product a·(b×c).
func outputDet3(arithmetic *Arithmetic, a, b, c Vec) *big.Rat {
	return arithmetic.Dot(a, arithmetic.Cross(b, c))
}

// OutputPlanarWall reports whether the wall from low at height 0 to high at
// height z is planar.
func OutputPlanarWall(low, high sectionrecord.LineSeg, z *big.Rat) bool {
	a, okA := PointOf(low.Start)
	b, okB := PointOf(low.End)
	c, okC := PointOf(high.Start)
	d, okD := PointOf(high.End)
	if !okA || !okB || !okC || !okD {
		return false
	}
	p, q, r, s := OutputP2(a, big.NewRat(0, 1)), OutputP2(b, big.NewRat(0, 1)),
		OutputP2(c, z), OutputP2(d, z)
	arithmetic := &Arithmetic{OK: true}
	det := outputDet3(arithmetic, outputDelta(arithmetic, p, q),
		outputDelta(arithmetic, p, r), outputDelta(arithmetic, p, s))
	return arithmetic.OK && det.Sign() == 0
}

func outputQuadPlanarConvex(q [4]OutputPoint) bool {
	arithmetic := &Arithmetic{OK: true}
	if outputDet3(arithmetic, outputDelta(arithmetic, q[0], q[1]),
		outputDelta(arithmetic, q[0], q[2]),
		outputDelta(arithmetic, q[0], q[3])).Sign() != 0 || !arithmetic.OK ||
		OutputCrossZero([3]OutputPoint{q[0], q[1], q[2]}) ||
		OutputCrossZero([3]OutputPoint{q[0], q[2], q[3]}) {
		return false
	}
	n0 := arithmetic.Cross(outputDelta(arithmetic, q[0], q[1]), outputDelta(arithmetic, q[0], q[2]))
	n1 := arithmetic.Cross(outputDelta(arithmetic, q[0], q[2]), outputDelta(arithmetic, q[0], q[3]))
	dot := arithmetic.Dot(n0, n1)
	// Monotone height and ordered edge parameters then make this a simple
	// planar quadrilateral; matching normals establish both triangle sides.
	return arithmetic.OK && dot.Sign() > 0
}

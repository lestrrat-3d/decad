package loftcontact

import (
	"context"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/loftrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/provenance"
	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

// QuarterHeldSeam is a private reading of one plane through the original
// authenticated Loft mesh. It certifies no true source seam or Boolean result.
type QuarterHeldSeam struct {
	Reason   string
	Cell     int
	Side     uint8
	Triangle int
	Stations int
	Pieces   int
	BottomX  *big.Rat
	TopX     *big.Rat
}

type heldPoint struct{ x, y, z *big.Rat }
type heldPiece struct {
	lo, hi heldPoint
	cell   int
	side   uint8
	tri    int
}
type heldCell struct{ tri [2]int }

func (m loftHullMath) quotient(a, b *big.Rat) (*big.Rat, error) {
	if err := m.step(); err != nil {
		return nil, err
	}
	if b.Sign() == 0 || a.Num().BitLen()+b.Denom().BitLen() > loftHullRatBits ||
		a.Denom().BitLen()+b.Num().BitLen() > loftHullRatBits {
		return nil, errLoftHullSize
	}
	r := new(big.Rat).Quo(a, b)
	if !loftHullFits(r) {
		return nil, errLoftHullSize
	}
	return r, nil
}

func heldFloatPoint(v r3.Vec) (heldPoint, bool) {
	// An exact binary64 lift has at most 1,075 bits in either part. Raw
	// coordinate comparisons below therefore allocate under 2,150 bits.
	x, ok := proofbound.RatOf(v.X)
	if !ok || !loftHullFits(x) {
		return heldPoint{}, false
	}
	y, ok := proofbound.RatOf(v.Y)
	if !ok || !loftHullFits(y) {
		return heldPoint{}, false
	}
	z, ok := proofbound.RatOf(v.Z)
	if !ok || !loftHullFits(z) {
		return heldPoint{}, false
	}
	return heldPoint{x, y, z}, true
}

func (m loftHullMath) heldEdgeAtY(a, b heldPoint, y *big.Rat) (heldPoint, bool, error) {
	lo, err := m.cmp(a.y, y)
	if err != nil {
		return heldPoint{}, false, err
	}
	hi, err := m.cmp(b.y, y)
	if err != nil {
		return heldPoint{}, false, err
	}
	if lo == 0 || hi == 0 || lo == hi {
		return heldPoint{}, false, nil
	}
	num, err := m.sub(y, a.y)
	if err != nil {
		return heldPoint{}, false, err
	}
	den, err := m.sub(b.y, a.y)
	if err != nil {
		return heldPoint{}, false, err
	}
	t, err := m.quotient(num, den)
	if err != nil {
		return heldPoint{}, false, err
	}
	dx, err := m.sub(b.x, a.x)
	if err != nil {
		return heldPoint{}, false, err
	}
	dx, err = m.mul(t, dx)
	if err != nil {
		return heldPoint{}, false, err
	}
	x, err := m.add(a.x, dx)
	if err != nil {
		return heldPoint{}, false, err
	}
	dz, err := m.sub(b.z, a.z)
	if err != nil {
		return heldPoint{}, false, err
	}
	dz, err = m.mul(t, dz)
	if err != nil {
		return heldPoint{}, false, err
	}
	z, err := m.add(a.z, dz)
	if err != nil {
		return heldPoint{}, false, err
	}
	return heldPoint{x, y, z}, true, nil
}

func (m loftHullMath) heldTriangleAtY(v [3]heldPoint, y *big.Rat) (heldPiece, bool, error) {
	var ends [2]heldPoint
	n := 0
	for edge := range 3 {
		if err := m.step(); err != nil {
			return heldPiece{}, false, err
		}
		a, b := v[edge], v[(edge+1)%3]
		if a.y.Cmp(y) == 0 || b.y.Cmp(y) == 0 {
			return heldPiece{}, false, nil // vertex or coplanar edge
		}
		p, crosses, err := m.heldEdgeAtY(a, b, y)
		if err != nil {
			return heldPiece{}, false, err
		}
		if !crosses {
			continue
		}
		if n >= len(ends) {
			return heldPiece{}, false, nil
		}
		ends[n] = p
		n++
	}
	if n != 2 {
		return heldPiece{}, false, nil
	}
	order, err := m.cmp(ends[0].z, ends[1].z)
	if err != nil {
		return heldPiece{}, false, err
	}
	if order == 0 {
		return heldPiece{}, false, nil
	}
	if order > 0 {
		ends[0], ends[1] = ends[1], ends[0]
	}
	return heldPiece{lo: ends[0], hi: ends[1]}, true, nil
}

func (m loftHullMath) heldProjectionSign(v [3]heldPoint) (int, error) {
	dy1, err := m.sub(v[1].y, v[0].y)
	if err != nil {
		return 0, err
	}
	dz1, err := m.sub(v[1].z, v[0].z)
	if err != nil {
		return 0, err
	}
	dy2, err := m.sub(v[2].y, v[0].y)
	if err != nil {
		return 0, err
	}
	dz2, err := m.sub(v[2].z, v[0].z)
	if err != nil {
		return 0, err
	}
	a, err := m.mul(dy1, dz2)
	if err != nil {
		return 0, err
	}
	b, err := m.mul(dz1, dy2)
	if err != nil {
		return 0, err
	}
	c, err := m.sub(a, b)
	if err != nil {
		return 0, err
	}
	return c.Sign(), nil
}

func (m loftHullMath) heldCapEdgeCount(tris [][3]int, a, b int) (int, error) {
	count := 0
	for _, tri := range tris {
		if err := m.step(); err != nil {
			return 0, err
		}
		for edge := range 3 {
			if err := m.step(); err != nil {
				return 0, err
			}
			left, right := tri[edge], tri[(edge+1)%3]
			if left == a && right == b || left == b && right == a {
				count++
			}
		}
	}
	return count, nil
}

// CertifyQuarterHeldSeam reads one y-plane crossing of a live original Loft.
// The caller supplies the real carrier's y coordinate and one shared output
// budget. A false result carries the first bounded refusal in Reason.
func CertifyQuarterHeldSeam(ctx context.Context, budget *proofbound.WorkBudget,
	loft *topo.Body, arcSegment int, carrierY float64) (QuarterHeldSeam, bool, error) {
	report := QuarterHeldSeam{Cell: -1, Triangle: -1}
	decline := func(reason string) (QuarterHeldSeam, bool, error) {
		report.Reason = reason
		return report, false, nil
	}
	if err := ctx.Err(); err != nil {
		return report, false, err
	}
	if budget == nil {
		return decline("budget")
	}
	if err := budget.Err(); err != nil {
		return report, false, err
	}
	if loft == nil || loft.Doc == nil || loft.Owner == nil || loft.Origin.Role != provenance.RoleBody {
		return decline("live_original")
	}
	loft.Doc.LiveMu.RLock()
	if len(loft.Doc.LiveSet) > 65_536 {
		loft.Doc.LiveMu.RUnlock()
		return decline("live_set_limit")
	}
	liveSet := append([]*topo.Body(nil), loft.Doc.LiveSet...)
	loft.Doc.LiveMu.RUnlock()
	live := false
	for _, candidate := range liveSet {
		if err := budget.Step(); err != nil {
			return report, false, err
		}
		if candidate == loft {
			live = true
			break
		}
	}
	if !live {
		return decline("live_original")
	}
	pl, payloadOK := loft.Payload.(loftrecord.Payload)
	if !payloadOK || pl.SourceIdentity == nil || pl.SourceIdentity.Origin != loft.Origin.Producer {
		return decline("source_identity")
	}
	if len(pl.Verts) == 0 || len(pl.Verts) > 512 || len(pl.Tris) == 0 || len(pl.Tris) > 512 {
		return decline("facet_limit")
	}
	if !pl.AuthenticatedReconstruction || pl.SurfaceResult || pl.ThroughBoreCut || pl.AnalyticBore ||
		pl.Xform != r3.Identity() || pl.ConstructionDelta != 0 || len(pl.BlendSegs) != 0 ||
		len(pl.ConstructionArcUpper) != 0 || len(pl.ConstructionLengthExtra) != 0 {
		return decline("source_identity")
	}
	if pl.Frame0.U() != (r3.Vec{X: 1}) || pl.Frame0.V() != (r3.Vec{Y: 1}) ||
		pl.Frame1.U() != (r3.Vec{X: 1}) || pl.Frame1.V() != (r3.Vec{Y: 1}) ||
		pl.Plane0.Origin.X != 0 || pl.Plane0.Origin.Y != 0 ||
		pl.Plane1.Origin.X != 0 || pl.Plane1.Origin.Y != 0 ||
		pl.Plane0.Origin.Z >= pl.Plane1.Origin.Z {
		return decline("source_frame")
	}
	if len(pl.Profile0.Outer.Segments) != len(pl.Profile1.Outer.Segments) ||
		len(pl.Profile0.Holes) != 0 || len(pl.Profile1.Holes) != 0 ||
		arcSegment < 0 || arcSegment >= len(pl.Profile0.Outer.Segments) {
		return decline("source_profile")
	}
	arc0, ok0 := pl.Profile0.Outer.Segments[arcSegment].(sectionrecord.ArcSeg)
	arc1, ok1 := pl.Profile1.Outer.Segments[arcSegment].(sectionrecord.ArcSeg)
	if !ok0 || !ok1 || arc0.Center != (sectionrecord.Point2{}) ||
		arc1.Center != (sectionrecord.Point2{}) ||
		arc0.Start.V != 0 || arc1.Start.V != 0 || arc0.Start.U <= 0 || arc1.Start.U <= 0 ||
		arc0.End.U != 0 || arc1.End.U != 0 || arc0.End.V <= 0 || arc1.End.V <= 0 {
		return decline("arc_role")
	}
	if pl.Walls < 0 || pl.Walls > len(pl.Tris) || len(pl.Cell) != pl.Walls ||
		len(pl.Side) != pl.Walls || len(pl.Segment) != 1 ||
		len(pl.Segment[0]) != pl.Walls/2 || pl.Walls%2 != 0 ||
		pl.CapStartCount < 0 || pl.CapStartCount > len(pl.Tris)-pl.Walls {
		return decline("wall_roles")
	}
	y, ok := proofbound.RatOf(carrierY)
	if !ok || !loftHullFits(y) {
		return decline("carrier")
	}
	m := loftHullMath{budget: budget}
	points := make([]heldPoint, len(pl.Verts))
	for i, v := range pl.Verts {
		if err := m.step(); err != nil {
			return report, false, err
		}
		p, valid := heldFloatPoint(v)
		if !valid {
			return decline("held_vertex")
		}
		points[i] = p
	}
	cells := make([]heldCell, len(pl.Segment[0]))
	for i := range cells {
		cells[i].tri = [2]int{-1, -1}
	}
	for i := range pl.Walls {
		if err := m.step(); err != nil {
			return report, false, err
		}
		role := pl.Cell[i]
		if role[0] != 0 || role[1] < 0 || role[1] >= len(cells) || pl.Side[i] > 1 ||
			cells[role[1]].tri[pl.Side[i]] >= 0 {
			report.Triangle = i
			return decline("wall_role")
		}
		cells[role[1]].tri[pl.Side[i]] = i
	}
	first, last := -1, -1
	for i, segment := range pl.Segment[0] {
		if err := m.step(); err != nil {
			return report, false, err
		}
		if cells[i].tri[0] < 0 || cells[i].tri[1] < 0 {
			report.Cell = i
			return decline("cell_roles")
		}
		if segment == arcSegment {
			if first < 0 {
				first = i
			}
			last = i
		}
	}
	if first < 0 {
		return decline("arc_cells")
	}
	for i := first; i <= last; i++ {
		if pl.Segment[0][i] != arcSegment {
			report.Cell = i
			return decline("arc_cell_order")
		}
	}
	report.Stations = last - first + 2
	var pieces []heldPiece
	var bottom, top heldPoint
	var prevBottom, prevTop int
	projectionSign := 0
	for i := first; i <= last; i++ {
		if err := m.step(); err != nil {
			return report, false, err
		}
		report.Cell = i
		aIdx, bIdx := cells[i].tri[0], cells[i].tri[1]
		a, b := pl.Tris[aIdx], pl.Tris[bIdx]
		if a[0] != b[0] || a[2] != b[1] {
			return decline("paired_triangles")
		}
		for _, idx := range [4]int{a[0], a[1], a[2], b[2]} {
			if idx < 0 || idx >= len(points) {
				return decline("triangle_index")
			}
		}
		bj, bnext, tnext, tj := points[a[0]], points[a[1]], points[a[2]], points[b[2]]
		if i == first {
			z0, valid := proofbound.RatOf(pl.Plane0.Origin.Z)
			z1, valid1 := proofbound.RatOf(pl.Plane1.Origin.Z)
			if !valid || !valid1 || pl.Verts[a[0]].X != arc0.Start.U || pl.Verts[a[0]].Y != arc0.Start.V ||
				pl.Verts[b[2]].X != arc1.Start.U || pl.Verts[b[2]].Y != arc1.Start.V {
				return decline("first_station")
			}
			bottom, top = bj, tj
			if bj.z.Cmp(z0) != 0 || tj.z.Cmp(z1) != 0 {
				return decline("cap_height")
			}
		} else if a[0] != prevBottom || b[2] != prevTop {
			return decline("station_link")
		}
		if bnext.y.Cmp(bj.y) <= 0 || tnext.y.Cmp(tj.y) <= 0 ||
			bj.z.Cmp(bnext.z) != 0 || tj.z.Cmp(tnext.z) != 0 {
			return decline("station_order")
		}
		if i == last {
			if pl.Verts[a[1]].X != arc0.End.U || pl.Verts[a[1]].Y != arc0.End.V ||
				pl.Verts[a[2]].X != arc1.End.U || pl.Verts[a[2]].Y != arc1.End.V {
				return decline("last_station")
			}
		}
		prevBottom, prevTop = a[1], a[2]
		for side, triIndex := range cells[i].tri {
			report.Side, report.Triangle = uint8(side), triIndex
			tri := pl.Tris[triIndex]
			v := [3]heldPoint{points[tri[0]], points[tri[1]], points[tri[2]]}
			sign, err := m.heldProjectionSign(v)
			if err != nil {
				return report, false, err
			}
			if sign == 0 || projectionSign != 0 && sign != projectionSign {
				return decline("projection")
			}
			projectionSign = sign
			minY, maxY := v[0].y, v[0].y
			for _, p := range v[1:] {
				if p.y.Cmp(minY) < 0 {
					minY = p.y
				}
				if p.y.Cmp(maxY) > 0 {
					maxY = p.y
				}
			}
			if minY.Cmp(y) >= 0 || maxY.Cmp(y) <= 0 {
				continue
			}
			piece, crossed, err := m.heldTriangleAtY(v, y)
			if err != nil {
				return report, false, err
			}
			if !crossed {
				return decline("triangle_crossing")
			}
			piece.cell, piece.side, piece.tri = i, uint8(side), triIndex
			pieces = append(pieces, piece)
		}
	}
	if len(pieces) == 0 {
		return decline("empty_crossing")
	}
	// Insertion sorting charges every exact comparison to the same output wall.
	for i := 1; i < len(pieces); i++ {
		for j := i; j > 0; j-- {
			cmp, err := m.cmp(pieces[j-1].lo.z, pieces[j].lo.z)
			if err != nil {
				return report, false, err
			}
			if cmp <= 0 {
				break
			}
			pieces[j-1], pieces[j] = pieces[j], pieces[j-1]
		}
	}
	report.Pieces = len(pieces)
	for i := 1; i < len(pieces); i++ {
		if err := m.step(); err != nil {
			return report, false, err
		}
		zCmp, err := m.cmp(pieces[i-1].hi.z, pieces[i].lo.z)
		if err != nil {
			return report, false, err
		}
		xCmp, err := m.cmp(pieces[i-1].hi.x, pieces[i].lo.x)
		if err != nil {
			return report, false, err
		}
		if zCmp != 0 || xCmp != 0 || pieces[i-1].tri == pieces[i].tri {
			report.Cell, report.Side, report.Triangle = pieces[i].cell, pieces[i].side, pieces[i].tri
			return decline("coverage")
		}
	}
	// The original cap boundary edges and wall pieces must share endpoints.
	capBottom, capTop := 0, 0
	for i := first; i <= last; i++ {
		if err := m.step(); err != nil {
			return report, false, err
		}
		a, b := pl.Tris[cells[i].tri[0]], pl.Tris[cells[i].tri[1]]
		p, crossed, err := m.heldEdgeAtY(points[a[0]], points[a[1]], y)
		if err != nil {
			return report, false, err
		}
		if crossed {
			incidence, countErr := m.heldCapEdgeCount(
				pl.Tris[pl.Walls:pl.Walls+pl.CapStartCount], a[0], a[1])
			if countErr != nil {
				return report, false, countErr
			}
			if incidence != 1 {
				report.Cell = i
				return decline("bottom_cap_edge")
			}
			capBottom++
			bottom = p
		}
		p, crossed, err = m.heldEdgeAtY(points[b[2]], points[a[2]], y)
		if err != nil {
			return report, false, err
		}
		if crossed {
			incidence, countErr := m.heldCapEdgeCount(
				pl.Tris[pl.Walls+pl.CapStartCount:], b[2], a[2])
			if countErr != nil {
				return report, false, countErr
			}
			if incidence != 1 {
				report.Cell = i
				return decline("top_cap_edge")
			}
			capTop++
			top = p
		}
	}
	if capBottom != 1 || capTop != 1 {
		return decline("cap_endpoint")
	}
	bottomX, err := m.cmp(bottom.x, pieces[0].lo.x)
	if err != nil {
		return report, false, err
	}
	bottomZ, err := m.cmp(bottom.z, pieces[0].lo.z)
	if err != nil {
		return report, false, err
	}
	topX, err := m.cmp(top.x, pieces[len(pieces)-1].hi.x)
	if err != nil {
		return report, false, err
	}
	topZ, err := m.cmp(top.z, pieces[len(pieces)-1].hi.z)
	if err != nil {
		return report, false, err
	}
	if bottomX != 0 || bottomZ != 0 || topX != 0 || topZ != 0 {
		return decline("cap_endpoint")
	}
	report.BottomX, report.TopX = bottom.x, top.x
	report.Reason = ""
	if err := budget.Err(); err != nil {
		return report, false, err
	}
	return report, true, nil
}

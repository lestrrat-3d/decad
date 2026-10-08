package loftmesh

import (
	"math/big"

	"github.com/lestrrat-3d/decad/internal/meshbool"
	"github.com/lestrrat-3d/decad/internal/proof"
)

// LoftCapCondition names the first condition of CapFamilyProof that one cap
// failed, or LoftCapProofHolds when every condition holds.
type LoftCapCondition int

const (
	// LoftCapProofHolds: every condition holds, and the cap's pairs are
	// decided.
	LoftCapProofHolds LoftCapCondition = iota
	// LoftCapStructure: the index structure the proof reads is not the one a
	// loft assembles (CapFamilyProof's (s)).
	LoftCapStructure
	// LoftCapOffPlane: a loop vertex has a nonzero exact sign against the
	// cap's plane (a).
	LoftCapOffPlane
	// LoftCapOtherSide: an opposite-cap vertex lies on the plane, or the
	// opposite cap's vertices lie on both sides of it (b).
	LoftCapOtherSide
	// LoftCapOrientation: a cap triangle is not positively oriented against
	// the plane's normal (c).
	LoftCapOrientation
	// LoftCapBoundary: the cap's directed edges do not net to its loops (d).
	LoftCapBoundary
	// LoftCapLoopArea: the loops' oriented areas are not one outer loop and
	// holes (e).
	LoftCapLoopArea
)

func (c LoftCapCondition) String() string {
	switch c {
	case LoftCapProofHolds:
		return "holds"
	case LoftCapStructure:
		return "(s) index structure"
	case LoftCapOffPlane:
		return "(a) loop vertex off the cap plane"
	case LoftCapOtherSide:
		return "(b) opposite cap not strictly on one side"
	case LoftCapOrientation:
		return "(c) cap triangle orientation"
	case LoftCapBoundary:
		return "(d) directed edges do not net to the loops"
	case LoftCapLoopArea:
		return "(e) loop orientation"
	default:
		return "unknown"
	}
}

const (
	capRoleNone uint8 = iota
	capRoleLoop
	capRoleOther
)

// CapFamilyProof decides one cap's triangle family (which 0: tris[Walls :
// Walls+CapStartCount], which 1: the rest) as a whole, from exact signs alone
// (docs/loft-design.md §6). Write C for the cap's triangles, L for its loops,
// O for the other cap's loop vertices, Π for the exact plane of C's first
// triangle and n for that triangle's exact normal. The conditions, tested in
// this order:
//
//   - (s) the index structure is the one Assemble builds: every loop has at
//     least three vertices, no vertex index appears twice across L and O, no
//     vertex of C is in O and no vertex of the other cap is in L, every wall
//     triangle has at least one vertex in O and its other vertices in L, its
//     L vertices are one vertex or the two ends of one loop edge, and every
//     loop edge is an edge of some wall triangle;
//   - (a) every vertex of L, and every vertex of every triangle of C, has
//     exact sign 0 against Π;
//   - (b) every vertex of O, and every vertex of every triangle of the other
//     cap, has a nonzero sign against Π, all the same;
//   - (c) every triangle of C has n · ((B − A) × (C − A)) > 0;
//   - (d) the directed-edge multiset of C nets to exactly the edges of L, each
//     loop traversed in one consistent direction, every other edge netting to
//     zero;
//   - (e) in the projection meshbool.ProjAxes(n), exactly one loop's oriented
//     signed area (its exact shoelace sign times its (d) direction) has the
//     triangles' orientation sign and every other loop's has the opposite
//     sign.
//
// PRECONDITION: S6 passed, so every triangle is noncollapsed. The proof
// holds only when the wall-wall pairs pass the pair audit, which proves every
// loop edge (a wall edge, by (s)) meets every other wall edge only as Table C
// expects, so the loops of L are simple and pairwise disjoint. Given that:
// (c)+(d) make Σ_T 1_T equal the winding number w_L of the oriented boundary
// almost everywhere (a 2-chain and the region chain with the same boundary
// differ by a 2-cycle, which is zero in the plane); (e) with simple disjoint
// loops makes w_L ∈ {0, 1}; so the triangles of C are interior disjoint and
// cover the polygon exactly. Netting on vertex INDICES leaves an unmatched
// edge wherever a vertex sits inside another triangle's edge, so (d) also
// rules that out, and it is also what keeps a vertex of C outside L (one
// Assemble never emits, and (a) still holds on Π) off the polygon's
// boundary. A cap triangle therefore meets a polygon edge only in
// shared vertices or as that edge, and two triangles of C meet exactly in
// their shared edge or vertex. A wall triangle U meets Π in exactly its L
// vertices — (b) puts its O vertices strictly on one side — which by (s) are
// one polygon vertex or one polygon edge, so T ∩ U = T ∩ (U ∩ Π) is exactly
// the contact their shared indices expect, for every cap triangle T. (b) also
// proves the two caps disjoint, since C lies in Π by (a) and the other cap's
// triangles are hulls of vertices (b) puts strictly on one side. So every pair with a triangle in C is
// admitted by the reference path.
//
// A failed condition admits nothing: the caller tests that cap's pairs
// pairwise.
func CapFamilyProof(data *LoftAuditData, tris [][3]int, s LoftAuditStructure, which int) LoftCapCondition {
	f := len(tris)
	if s.Walls < 0 || s.CapStartCount < 0 || s.Walls+s.CapStartCount > f {
		return LoftCapStructure
	}
	from, to := s.Walls, s.Walls+s.CapStartCount
	otherFrom, otherTo := to, f
	loops, other := s.Loops0, s.Loops1
	if which == 1 {
		from, to, otherFrom, otherTo = otherFrom, otherTo, from, to
		loops, other = other, loops
	}
	if to <= from || len(loops) == 0 {
		return LoftCapStructure
	}

	nv := len(data.Xverts)
	role := make([]uint8, nv)
	next := make([]int, nv)
	for _, loop := range loops {
		if len(loop) < 3 {
			return LoftCapStructure
		}
		for k, v := range loop {
			if v < 0 || v >= nv || role[v] != capRoleNone {
				return LoftCapStructure
			}
			role[v] = capRoleLoop
			next[v] = loop[(k+1)%len(loop)]
		}
	}
	for _, loop := range other {
		for _, v := range loop {
			if v < 0 || v >= nv || role[v] != capRoleNone {
				return LoftCapStructure
			}
			role[v] = capRoleOther
		}
	}
	if !capStructureHolds(tris, s.Walls, from, to, otherFrom, otherTo, role, next, loops) {
		return LoftCapStructure
	}

	// (a) and (b), against the exact plane of the cap's first triangle.
	plane := data.Planes[from]
	var sum, term big.Int
	onPlane := func(v int) bool { return plane.Sign(data.Xverts[v], &sum, &term) == 0 }
	for _, loop := range loops {
		for _, v := range loop {
			if !onPlane(v) {
				return LoftCapOffPlane
			}
		}
	}
	for t := from; t < to; t++ {
		for _, v := range tris[t] {
			if !onPlane(v) {
				return LoftCapOffPlane
			}
		}
	}
	side := 0
	oneSide := func(v int) bool {
		sign := plane.Sign(data.Xverts[v], &sum, &term)
		if sign == 0 || (side != 0 && sign != side) {
			return false
		}
		side = sign
		return true
	}
	for _, loop := range other {
		for _, v := range loop {
			if !oneSide(v) {
				return LoftCapOtherSide
			}
		}
	}
	for t := otherFrom; t < otherTo; t++ {
		for _, v := range tris[t] {
			if !oneSide(v) {
				return LoftCapOtherSide
			}
		}
	}
	if side == 0 {
		return LoftCapOtherSide
	}

	// (c), and the directed-edge multiset (d) reads.
	n := data.Norms[from]
	net := make(map[[2]int]int, 3*(to-from))
	for t := from; t < to; t++ {
		if proof.XdotSign(n, data.Norms[t]) <= 0 {
			return LoftCapOrientation
		}
		tri := tris[t]
		for k := range 3 {
			a, b := tri[k], tri[(k+1)%3]
			if a < b {
				net[[2]int{a, b}]++
			} else {
				net[[2]int{b, a}]--
			}
		}
	}

	// (d): each loop edge nets to ±1, one direction per loop; the rest to 0.
	dirs := make([]int, len(loops))
	for li, loop := range loops {
		for k, a := range loop {
			b := loop[(k+1)%len(loop)]
			key, want := [2]int{a, b}, 1
			if a > b {
				key, want = [2]int{b, a}, -1
			}
			d := net[key] * want
			if d != 1 && d != -1 {
				return LoftCapBoundary
			}
			if dirs[li] != 0 && dirs[li] != d {
				return LoftCapBoundary
			}
			dirs[li] = d
			delete(net, key)
		}
	}
	for _, c := range net {
		if c != 0 {
			return LoftCapBoundary
		}
	}

	// (e): the triangles' orientation sign in the projection, against each
	// loop's oriented shoelace sign.
	projected := data.Projections[from]
	sigma := meshbool.Cross2xSign(projected[0], projected[1], projected[2])
	if sigma == 0 {
		return LoftCapLoopArea
	}
	u, v := meshbool.ProjAxes(n)
	outer := 0
	for li, loop := range loops {
		oriented := capLoopAreaSign(data, loop, u, v) * dirs[li]
		switch oriented {
		case sigma:
			outer++
		case -sigma:
		default:
			return LoftCapLoopArea
		}
	}
	if outer != 1 {
		return LoftCapLoopArea
	}
	return LoftCapProofHolds
}

// capStructureHolds is CapFamilyProof's (s) over the triangle index sets.
func capStructureHolds(tris [][3]int, walls, from, to, otherFrom, otherTo int, role []uint8, next []int, loops [][]int) bool {
	for t := from; t < to; t++ {
		for _, v := range tris[t] {
			if role[v] == capRoleOther {
				return false
			}
		}
	}
	for t := otherFrom; t < otherTo; t++ {
		for _, v := range tris[t] {
			if role[v] == capRoleLoop {
				return false
			}
		}
	}
	// hit[v] records that the loop edge (v, next[v]) is a wall edge.
	hit := make([]bool, len(role))
	for t := range walls {
		var inLoop [3]int
		nLoop, nOther := 0, 0
		for _, v := range tris[t] {
			switch role[v] {
			case capRoleLoop:
				inLoop[nLoop] = v
				nLoop++
			case capRoleOther:
				nOther++
			default:
				return false
			}
		}
		if nOther == 0 {
			return false
		}
		if nLoop != 2 {
			continue
		}
		a, b := inLoop[0], inLoop[1]
		switch {
		case next[a] == b:
			hit[a] = true
		case next[b] == a:
			hit[b] = true
		default:
			return false
		}
	}
	for _, loop := range loops {
		for _, v := range loop {
			if !hit[v] {
				return false
			}
		}
	}
	return true
}

// capLoopAreaSign is the sign of one loop's exact shoelace area in the
// projection onto axes (u, v), in the loop's own vertex order.
func capLoopAreaSign(data *LoftAuditData, loop []int, u, v int) int {
	area := new(big.Rat)
	var cross, term big.Rat
	for k, a := range loop {
		pa, pb := data.Xverts[a], data.Xverts[loop[(k+1)%len(loop)]]
		au, av := meshbool.RatCoordOf(pa, u), meshbool.RatCoordOf(pa, v)
		bu, bv := meshbool.RatCoordOf(pb, u), meshbool.RatCoordOf(pb, v)
		cross.Mul(au, bv)
		term.Mul(bu, av)
		cross.Sub(&cross, &term)
		area.Add(area, &cross)
	}
	return area.Sign()
}

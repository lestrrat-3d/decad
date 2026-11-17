package retainedblend

import (
	"context"
	"errors"
	"fmt"
	"math"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/decad/internal/topobuild"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads a C3 arc end (agent-docs/boss-root-overflow-design.md §5.5):
// the corner where the arc's wall B, the cliff cylinder G below the plane and
// the full-height side face K meet the plane A, with G and K tangent to B along
// the generator through the end. A tangency is never decided here: the caller
// proves it from the records the faces came from and hands the proof in.

// TangentEnds proves C3 arc ends from records the blend cannot read
// (agent-docs/boss-root-overflow-design.md §5.5.2).
type TangentEnds interface {
	// Prove returns the record proof that b, g and k (faces of the body the
	// blend reads) are G1 at one recorded point with the levels of a C3
	// corner, and false when the record proves no such corner.
	Prove(b, g, k *topo.Face) (TangentProof, bool)
}

// TangentProof is one proven C3 corner.
type TangentProof struct {
	// At is the recorded joint V_r, placed in the coordinates of the body the
	// blend reads. Only reject-only cross-checks read it.
	At r3.Vec
	// Turn is θ_up, an upper bound on the K → G turn at V_r in radians: zero
	// for an exactly tangent joint.
	Turn float64
}

// FilletTangent is Fillet with the proofs of C3 arc ends: a selected arc whose
// end is a four-face corner that tangent proves (agent-docs/boss-root-overflow-design.md
// §5.5.2) is read as such a corner. A nil tangent reads none, and Fillet is
// FilletTangent with none.
func FilletTangent(ctx context.Context, env topo.Env, b *topo.Body, fp facetedrecord.Payload, edges []*topo.Edge,
	radius, radiusDelta float64, tangent TangentEnds) (*topo.Body, bool, error) {
	out, recognized, err := blend(ctx, env, b, fp, edges, blendSpec{fillet: true, r: radius, tangent: tangent},
		radiusDelta, "fillet")
	return publishCharged(ctx, env, out, recognized, err)
}

// tangentCorner is a four-face arc end that the prover proved: the faces of
// agent-docs/boss-root-overflow-design.md §5.5.1 and the record's proof.
type tangentCorner struct {
	a, b, g, k *topo.Face
	proof      TangentProof
}

// provenEnd reports whether e, an arc between a plane A and a wall B, ends at
// v in the four-face corner of agent-docs/boss-root-overflow-design.md §5.5.1
// and tangent proves (B, G, K) there. It is false for a nil tangent.
func provenEnd(tangent TangentEnds, all []*topo.Edge, e *topo.Edge, v *topo.Vertex) bool {
	_, ok := tangentCornerAt(tangent, all, e, v)
	return ok
}

// tangentCornerAt reads the corner of e at v from the topology: the faces of
// every edge that ends at v must be exactly A, B, G and K, with G the face
// that shares an edge with A at v other than B and K the remaining one.
func tangentCornerAt(tangent TangentEnds, all []*topo.Edge, e *topo.Edge, v *topo.Vertex) (tangentCorner, bool) {
	if tangent == nil || e == nil || v == nil || len(e.Faces) != 2 {
		return tangentCorner{}, false
	}
	var a, b *topo.Face
	for _, f := range e.Faces {
		if f == nil {
			return tangentCorner{}, false
		}
		if _, plane := f.Surface.(surfacegeom.Plane); plane {
			a = f
		} else {
			b = f
		}
	}
	if a == nil || b == nil || a == b {
		return tangentCorner{}, false
	}
	faces := map[*topo.Face]struct{}{}
	beside := map[*topo.Face]struct{}{} // faces that share an edge with a at v
	for _, c := range all {
		if c == nil || (c.Start != v && c.End != v) {
			continue
		}
		hasA := false
		for _, f := range c.Faces {
			if f == nil {
				return tangentCorner{}, false
			}
			faces[f] = struct{}{}
			hasA = hasA || f == a
		}
		if !hasA {
			continue
		}
		for _, f := range c.Faces {
			if f != a {
				beside[f] = struct{}{}
			}
		}
	}
	delete(beside, b)
	if len(faces) != 4 || len(beside) != 1 {
		return tangentCorner{}, false
	}
	var g, k *topo.Face
	for f := range beside {
		g = f
	}
	for f := range faces {
		if f != a && f != b && f != g {
			k = f
		}
	}
	if k == nil {
		return tangentCorner{}, false
	}
	proof, ok := tangent.Prove(b, g, k)
	if !ok {
		return tangentCorner{}, false
	}
	return tangentCorner{a: a, b: b, g: g, k: k, proof: proof}, true
}

// provenArc reports whether both ends of e are corners provenEnd proves.
func provenArc(tangent TangentEnds, all []*topo.Edge, e *topo.Edge) bool {
	return provenEnd(tangent, all, e, e.Start) && provenEnd(tangent, all, e, e.End)
}

// allEdges lists the edges of b when a tangent prover will read them, and
// none otherwise.
func allEdges(b *topo.Body, tangent TangentEnds) []*topo.Edge {
	if tangent == nil {
		return nil
	}
	return topobuild.Edges(b)
}

// tangentRead is what readArcTangent needs to read the C3 ends of one arc: the
// prover, the edges of the body the arc belongs to and the arc itself.
type tangentRead struct {
	tangent TangentEnds
	all     []*topo.Edge
	edge    *topo.Edge
}

// newTangentRead is nil when tangent is nil.
func newTangentRead(tangent TangentEnds, b *topo.Body, e *topo.Edge) *tangentRead {
	if tangent == nil {
		return nil
	}
	return &tangentRead{tangent: tangent, all: topobuild.Edges(b), edge: e}
}

// tangentRefusalError is a refusal the C3 reading names itself. The route reports
// it, where it leaves an arc it cannot read to the generic refusal.
type tangentRefusalError struct{ err error }

func (r tangentRefusalError) Error() string { return r.err.Error() }
func (r tangentRefusalError) Unwrap() error { return r.err }

func refuseTangent(msg string) error {
	return tangentRefusalError{err: fmt.Errorf("%w: %s", decaderr.ErrUnsupported, msg)}
}

func isTangentRefusal(err error) bool {
	var r tangentRefusalError
	return errors.As(err, &r)
}

// tangentEnd is the ideal end of a proven C3 corner
// (agent-docs/boss-root-overflow-design.md §5.5.3). The end's cylEnd reads G*,
// the circle of G's radius whose centre lies d' from the rim's axis along the
// direction of G's tag centre, which touches the wall's circle at T.
type tangentEnd struct {
	// external is set when G* touches B's circle from outside (d' = R + r_G).
	external bool
	// t is the tangent point T = O + R·û, where the wall's row meets G*.
	t r3.Vec
	// proof is the record's proof of the corner.
	proof TangentProof
	// eC is the charge part of the end's error, eR the rounding part and e their
	// sum; eD is the rounding of d alone. reach is e·(1 + 2R/d') + e_d.
	eC, eR, e, eD, reach float64
	// k is the full-height face the end's wall continues onto.
	k *topo.Face
	// dTag is the in-plane distance of G's tag centre from the rim's axis.
	dTag float64
	// sigmaMax is the largest σ the end's vertices carry (§5.5.6); chargeTangentEnds sets it.
	sigmaMax float64
}

// psiWall is ψ at the wall's row: 0 for an external end, π for an internal one.
func (t *tangentEnd) psiWall() float64 {
	if t.external {
		return 0
	}
	return math.Pi
}

// wallAzimuth is the azimuth of the wall's row on end c: the tangent point's,
// α = 0 without an acos, unwrapped to lie within π of ref.
func (c *cylEnd) wallAzimuth(ref float64) float64 {
	t := c.thetaC
	return t + 2*math.Pi*math.Round((ref-t)/(2*math.Pi))
}

// mixedTangent refuses an arc with one C3 end and one C2 end.
func (a arcRim) mixedTangent() error {
	if a.cyl[0] == nil || a.cyl[1] == nil || (a.cyl[0].tan == nil) == (a.cyl[1].tan == nil) {
		return nil
	}
	return refuseTangent("an arc with one tangent and one crossing cylinder end is not built")
}

// readTangentEnd reads end i at vertex as a proven C3 corner. ok is false when
// tr proves no corner there, and the caller keeps the refusal of cornerOf.
// Every refusal is a tangentRefusalError.
func (a *arcRim) readTangentEnd(i int, vertex *topo.Vertex, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	tr *tangentRead) (end lineEnd, cyl *cylEnd, ok bool, err error) {
	if tr == nil {
		return lineEnd{}, nil, false, nil
	}
	corner, ok := tangentCornerAt(tr.tangent, tr.all, tr.edge, vertex)
	if !ok {
		return lineEnd{}, nil, false, nil
	}
	if corner.a != a.planeFace || corner.b != a.wallFace {
		return lineEnd{}, nil, true, refuseTangent("an arc end's corner disagrees with the arc's faces")
	}
	v := -1
	for k, p := range fp.Verts {
		if p == vertex.Position {
			v = k
			break
		}
	}
	if v < 0 {
		return lineEnd{}, nil, true, refuseTangent("an edge end is not a vertex of the held mesh")
	}
	want := map[int]struct{}{}
	for _, f := range []*topo.Face{corner.a, corner.b, corner.g, corner.k} {
		fi, in := faceIndex[f]
		if !in {
			return lineEnd{}, nil, true, refuseTangent("an arc end's face is not a face of the held body")
		}
		want[fi] = struct{}{}
	}
	got := map[int]struct{}{}
	for k, t := range fp.Tris {
		if t[0] == v || t[1] == v || t[2] == v {
			got[fp.FaceOf[k]] = struct{}{}
		}
	}
	if len(want) != 4 || len(got) != 4 {
		return lineEnd{}, nil, true, refuseTangent("an edge end is not a corner of exactly four faces")
	}
	for fi := range want {
		if _, in := got[fi]; !in {
			return lineEnd{}, nil, true, refuseTangent("an edge end is not a corner of exactly four faces")
		}
	}
	end, cyl, err = a.tangentCylEnd(i, v, corner, fp)
	return end, cyl, true, err
}

// tangentCylEnd builds the ideal end of the proven corner at held vertex v
// (§5.5.3), uncharged.
func (a *arcRim) tangentCylEnd(i, v int, corner tangentCorner, fp facetedrecord.Payload) (lineEnd, *cylEnd, error) {
	g := corner.g
	notKind := refuseTangent("an arc end meets a face that is not a retained cylinder")
	cyl, isCyl := g.Surface.(surfacegeom.Cylinder)
	if !isCyl || !g.RetainedTag || g.TagDeparture != 0 || !wallRecord(g) {
		return lineEnd{}, nil, notKind
	}
	deltaG, _ := faceDelta(g)
	rG, err := cyl.Radius.In(units.Millimeter)
	if err != nil || !(rG > 0) || math.IsInf(rG, 0) {
		return lineEnd{}, nil, notKind
	}
	axis, okAxis := cyl.Axis.Normalize()
	if !okAxis {
		return lineEnd{}, nil, notKind
	}
	if !g.Reversed {
		return lineEnd{}, nil, refuseTangent("an arc end meets a cylinder whose material lies inside it")
	}
	eps := axis.Cross(a.n).Len()
	if !(eps <= parallelTol) {
		return lineEnd{}, nil, refuseTangent("an arc end meets a cylinder that is not parallel to the axis")
	}
	R := a.radius
	c := cyl.Origin.Add(axis.Scale(a.n.Dot(a.center.Sub(cyl.Origin)) / a.n.Dot(axis)))
	q := c.Sub(a.center)
	qu, qw := q.Dot(a.u), q.Dot(a.w)
	d := math.Hypot(qu, qw)
	slack := a.slack()
	if !(d > 64*slack) {
		return lineEnd{}, nil, refuseTangent("an arc end meets a cylinder coaxial with the wall")
	}
	// B reversed puts its material outside the circle: the corner round then
	// lies inside it. Otherwise G* touches from outside.
	external := !a.matOutside
	dPrime := R + rG
	if !external {
		if !(rG < R) {
			return lineEnd{}, nil, refuseTangent("an internal arc end needs a cylinder smaller than the wall")
		}
		dPrime = R - rG
	}
	// The charge part: the tags' displacements and the K to G turn.
	deltaB := a.wallDelta
	eC := proofbound.AbsSumUpper(deltaB, deltaG,
		proofbound.ProductUpper(proofbound.AbsSumUpper(rG, deltaG), corner.proof.Turn))
	sFar := farScale(a.scale, a.center, cyl.Origin, R, rG, d)
	eD := 16 * ulp * sFar
	if !(math.Abs(d-dPrime) <= eC+eD) {
		return lineEnd{}, nil, refuseTangent("an arc end's tags disagree with its proven tangency")
	}
	eR := proofbound.AbsSumUpper(math.Abs(d-dPrime), eD)
	e := proofbound.AbsSumUpper(eC, eR)
	reach := proofbound.AbsSumUpper(proofbound.ProductUpper(e, 1+2*R/dPrime), eD)
	dir := a.u.Scale(qu / d).Add(a.w.Scale(qw / d))
	tPoint := a.center.Add(dir.Scale(R))
	held := fp.Verts[v]
	allow := proofbound.AbsSumUpper(fp.VertexBound[v], slack, reach)
	if !(held.Sub(tPoint).Len() <= allow) || !(corner.proof.At.Sub(tPoint).Len() <= allow) {
		return lineEnd{}, nil, refuseTangent("an arc end is not where its three tags meet")
	}
	ce := &cylEnd{
		face: g, c: a.center.Add(dir.Scale(dPrime)), axis: axis, rG: rG, d: dPrime, sigma: 1, eps: eps, sFar: sFar,
		thetaC: a.sign * math.Atan2(qw, qu),
		tan: &tangentEnd{external: external, t: tPoint, proof: corner.proof, eC: eC, eR: eR, e: e, eD: eD,
			reach: reach, k: corner.k, dTag: d},
	}
	if i == 1 {
		ce.sigma = -1
	}
	return lineEnd{face: g, exact: tPoint, v: v}, ce, nil
}

// sigmaBound is the bound of a cap vertex on G*: σ's largest value at the end (§5.5.6),
// zero for an end that is not C3.
func (t *tangentEnd) sigmaBound() float64 {
	if t == nil {
		return 0
	}
	return t.sigmaMax
}

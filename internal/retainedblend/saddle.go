package retainedblend

import (
	"context"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/booleaneval"
	"github.com/lestrrat-3d/decad/internal/crossbore"
	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/facetedrecord"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// This file reads R7 of agent-docs/retained-edge-blend-design.md Table RR: the
// closed saddle edge where a retained bore crosses a larger retained cylinder
// at right angles, as a cross hole leaves a turned shaft. Chamfer cuts it
// (saddle_tool.go) and Fillet rewrites the mesh near it (saddle_fillet.go).

// saddleKey identifies a saddle: the pose its two tags and its mouth give, the
// two radii, the edge's Convex flag, and the side σ of each cylinder the
// rolling ball lies on (−1 inside, +1 outside; the larger cylinder first). Two
// selected edges with one key name one rim.
type saddleKey struct {
	pose   crossbore.Pose
	R, r   float64
	convex bool
	sigma  [2]int
}

// mouthSides is σ at the convex mouth of a cross hole: the ball inside the
// larger cylinder and outside the bore.
var mouthSides = [2]int{-1, 1}

// outsideSides is σ with the ball outside both cylinders: at a convex edge
// where two bores cross in a block, and at a concave one where a boss or pin
// roots on a rod.
var outsideSides = [2]int{1, 1}

// built reports whether the chamfer builds the saddle: the convex mouth of a
// cross hole, σ = (−1, +1) at a convex edge, so the shaft is not Reversed and
// the bore is. σ alone does not tell it: a concave pin root inside a tube's
// bore, both faces' Reversed flags and the Convex flag flipped together, has
// the same σ, and a blend there adds material.
func (k saddleKey) built() bool { return k.convex && k.sigma == mouthSides }

// filletBuilt reports whether the fillet builds the saddle
// (agent-docs/retained-edge-blend-design.md §4.10): the chamfer's mouth, and
// σ = (+1, +1) at either a convex edge (two crossing bores, both faces
// Reversed; the fillet removes material) or a concave one (a boss on a rod,
// neither Reversed; the fillet adds material). The concave pin root inside a
// tube's bore, σ = (−1, +1) at a concave edge, and every pattern with the
// ball inside the bore stay SR1.
func (k saddleKey) filletBuilt() bool {
	return k.built() || k.sigma == outsideSides
}

// saddle is one admitted saddle edge.
type saddle struct {
	key       saddleKey
	big, bore *topo.Face
	// chain is the held seam loop: the vertices the two walls' facets share at
	// this mouth.
	chain []int
	// betaBig and betaBore are the largest held vertex bounds of the two
	// walls' facets.
	betaBig, betaBore float64
	scale             float64
}

// cardinal returns the index of a unit coordinate axis: one component ±1 and
// the others exactly 0.
func cardinal(v r3.Vec) (int, bool) {
	axis := -1
	for i, x := range [3]float64{v.X, v.Y, v.Z} {
		switch x {
		case 0:
		case 1, -1:
			if axis >= 0 {
				return 0, false
			}
			axis = i
		default:
			return 0, false
		}
	}
	return axis, axis >= 0
}

func coord(v r3.Vec, i int) float64 { return [3]float64{v.X, v.Y, v.Z}[i] }

// ballSide is σ for a face of a blended edge: the ball lies in the material at
// a convex edge and in the void at a concave one, and a face's material lies
// inside its cylinder unless the face is Reversed.
func ballSide(f *topo.Face, convex bool) int {
	s := -1
	if f.Reversed {
		s = 1
	}
	if !convex {
		s = -s
	}
	return s
}

// saddleTags reads the two walls of a candidate saddle edge from their tags
// alone: two retained cylinders of different radii, the larger (radius R) and
// the bore (radius r), with unit cardinal axes on different coordinates that
// meet exactly, and 2r² < R² in rationals. σ comes from the Convex flag and
// each face's Reversed flag (ballSide). It returns the pose with the mouth
// side unset (Sign[0] = +1).
func saddleTags(e *topo.Edge) (shaft, bore *topo.Face, key saddleKey, ok bool) {
	if e == nil || e.Start == nil || e.Start != e.End || len(e.Faces) != 2 ||
		e.Faces[0] == nil || e.Faces[1] == nil || e.Faces[0] == e.Faces[1] {
		return nil, nil, saddleKey{}, false
	}
	var cyl [2]surfacegeom.Cylinder
	var radius [2]float64
	for i, f := range e.Faces {
		c, isCyl := f.Surface.(surfacegeom.Cylinder)
		if !isCyl || !f.RetainedTag || f.TagDeparture != 0 || !tagIsRecord(f) {
			return nil, nil, saddleKey{}, false
		}
		var err error
		if radius[i], err = c.Radius.In(units.Millimeter); err != nil || !(radius[i] > 0) || math.IsInf(radius[i], 0) {
			return nil, nil, saddleKey{}, false
		}
		cyl[i] = c
	}
	switch {
	case radius[0] > radius[1]:
		shaft, bore = e.Faces[0], e.Faces[1]
	case radius[1] > radius[0]:
		shaft, bore = e.Faces[1], e.Faces[0]
		cyl[0], cyl[1] = cyl[1], cyl[0]
		radius[0], radius[1] = radius[1], radius[0]
	default:
		return nil, nil, saddleKey{}, false
	}
	R, r := radius[0], radius[1]
	// 2r² < R², exactly: the opening stays one graph over the shaft's angle
	// only below r/R = 1/√2.
	r2 := new(big.Rat).Mul(proofarith.FloatRat(r), proofarith.FloatRat(r))
	if new(big.Rat).Add(r2, r2).Cmp(new(big.Rat).Mul(proofarith.FloatRat(R), proofarith.FloatRat(R))) >= 0 {
		return nil, nil, saddleKey{}, false
	}
	ib, okB := cardinal(cyl[0].Axis)
	ic, okC := cardinal(cyl[1].Axis)
	if !okB || !okC || ib == ic {
		return nil, nil, saddleKey{}, false
	}
	ik := 3 - ib - ic
	// The axes meet exactly when their origins agree on the third coordinate.
	if coord(cyl[0].Origin, ik) != coord(cyl[1].Origin, ik) {
		return nil, nil, saddleKey{}, false
	}
	var shift [3]float64
	shift[ib] = coord(cyl[1].Origin, ib)
	shift[ic] = coord(cyl[0].Origin, ic)
	shift[ik] = coord(cyl[0].Origin, ik)
	// Local X is the bore axis, local Y the third coordinate, local Z the
	// shaft axis.
	key = saddleKey{
		pose: crossbore.Pose{
			Axis:  [3]int{ic, ik, ib},
			Sign:  [3]float64{1, 1, 1},
			Shift: r3.Vec{X: shift[0], Y: shift[1], Z: shift[2]},
		},
		R: R, r: r,
		convex: e.Convex,
		sigma:  [2]int{ballSide(shaft, e.Convex), ballSide(bore, e.Convex)},
	}
	return shaft, bore, key, true
}

// looksLikeSaddle reports whether e is an R7 saddle the chamfer builds, from
// its tags and Convex flag alone (saddleKey.built).
func looksLikeSaddle(e *topo.Edge) bool {
	_, _, key, ok := saddleTags(e)
	return ok && key.built()
}

// local returns the local coordinates of world point p under pose, in float:
// a signed difference, rounded once.
func local(pose crossbore.Pose, p r3.Vec) r3.Vec {
	w := [3]float64{p.X, p.Y, p.Z}
	s := [3]float64{pose.Shift.X, pose.Shift.Y, pose.Shift.Z}
	var l [3]float64
	for i := range 3 {
		a := pose.Axis[i]
		l[i] = pose.Sign[i] * (w[a] - s[a])
	}
	return r3.Vec{X: l[0], Y: l[1], Z: l[2]}
}

// readSaddle reads a saddle edge that admit accepts against the held mesh:
// the seam loop the two walls share at the edge's start, the mouth it lies
// on, and the walls' held bounds. Every held seam vertex must lie within its bound plus slack of both
// cylinders and on one side of the shaft's axis; this falsifies the edge and
// never admits it.
func readSaddle(e *topo.Edge, fp facetedrecord.Payload, faceIndex map[*topo.Face]int,
	admit func(saddleKey) bool) (saddle, error) {
	shaft, bore, key, ok := saddleTags(e)
	if !ok || !admit(key) {
		return saddle{}, fmt.Errorf("%w: the saddle's walls changed", decaderr.ErrUnsupported)
	}
	s := saddle{key: key, big: shaft, bore: bore, scale: 1}
	for _, v := range fp.Verts {
		s.scale = max(s.scale, math.Abs(v.X), math.Abs(v.Y), math.Abs(v.Z))
	}
	ib, iBore := faceIndex[shaft], faceIndex[bore]
	for i, t := range fp.Tris {
		switch fp.FaceOf[i] {
		case ib:
			for _, v := range t {
				s.betaBig = max(s.betaBig, fp.VertexBound[v])
			}
		case iBore:
			for _, v := range t {
				s.betaBore = max(s.betaBore, fp.VertexBound[v])
			}
		}
	}
	loops, err := boundaryLoops(fp.Tris, facetsOfFace(fp.FaceOf, iBore))
	if err != nil {
		return saddle{}, err
	}
	onBig := map[int]struct{}{}
	for _, f := range facetsOfFace(fp.FaceOf, ib) {
		for _, v := range fp.Tris[f] {
			onBig[v] = struct{}{}
		}
	}
	start := e.Start.Position
	for _, loop := range loops {
		for _, v := range loop {
			if fp.Verts[v] == start {
				s.chain = loop
			}
		}
	}
	if s.chain == nil {
		return saddle{}, fmt.Errorf("%w: the saddle edge has no held seam loop", decaderr.ErrUnsupported)
	}
	slack := 64 * ulp * s.scale
	side := 0.0
	for _, v := range s.chain {
		if _, shared := onBig[v]; !shared {
			return saddle{}, fmt.Errorf("%w: the saddle's held seam leaves the shaft wall", decaderr.ErrUnsupported)
		}
		p := local(key.pose, fp.Verts[v])
		tol := fp.VertexBound[v] + slack
		if math.Abs(math.Hypot(p.X, p.Y)-key.R) > tol || math.Abs(math.Hypot(p.Y, p.Z)-key.r) > tol {
			return saddle{}, fmt.Errorf("%w: a held seam vertex is off the two cylinders the saddle's tags state",
				decaderr.ErrUnsupported)
		}
		switch {
		case side == 0 && p.X != 0:
			side = math.Copysign(1, p.X)
		case p.X*side <= 0:
			return saddle{}, fmt.Errorf("%w: the saddle's held seam crosses the shaft's axis", decaderr.ErrUnsupported)
		}
	}
	// The mouth lies on local +X: flip the bore axis, and the third axis with
	// it. The pose may be left-handed either way; nothing reads its hand, and
	// the tool's winding comes from its signed volume (placeSaddleTool).
	s.key.pose.Sign[0], s.key.pose.Sign[1] = side, side
	return s, nil
}

// SaddleChamfer bevels the selected saddle edges of b (Table RR R7): one mesh
// Boolean cuts b by a tool of one proven band-shaped ring per edge
// (agent-docs/retained-edge-blend-design.md §4.9). recognized is false when
// some selected edge is no saddle; the caller then tries the other routes.
// The span is the number of producer identities the result used: the tool's
// and the cut's.
func SaddleChamfer(ctx context.Context, env topo.Env, mesher booleaneval.OperandMesher, b *topo.Body,
	fp facetedrecord.Payload, edges []*topo.Edge, d, dDelta float64, asymmetric bool) (*topo.Body, int, bool, error) {
	if len(edges) == 0 || !b.Solid {
		return nil, 0, false, nil
	}
	for _, e := range edges {
		if !looksLikeSaddle(e) {
			return nil, 0, false, nil
		}
	}
	switch {
	case asymmetric:
		return nil, 0, true, fmt.Errorf("%w: a saddle chamfer takes one setback on both walls", decaderr.ErrUnsupported)
	case dDelta != 0:
		return nil, 0, true, fmt.Errorf("%w: a retained edge blend needs a size that converts exactly to millimetres",
			decaderr.ErrUnsupported)
	case proofbound.IsNonFinite(d) || !(d > 0):
		return nil, 0, true, fmt.Errorf("%w: a blend needs a positive size", decaderr.ErrDegenerate)
	case fp.Xform != r3.Identity():
		return nil, 0, true, fmt.Errorf("%w: a saddle chamfer reads an unplaced mesh only", decaderr.ErrUnsupported)
	}
	faceIndex := faceIndexOf(b)
	saddles := make([]saddle, len(edges))
	for i, e := range edges {
		s, err := readSaddle(e, fp, faceIndex, saddleKey.built)
		if err != nil {
			return nil, 0, true, err
		}
		if d > s.key.r/2 {
			return nil, 0, true, fmt.Errorf("%w: a saddle chamfer's setback is past half the bore's radius",
				decaderr.ErrUnsupported)
		}
		for _, earlier := range saddles[:i] {
			if earlier.key == s.key {
				return nil, 0, true, fmt.Errorf("%w: the selection names one rim twice", decaderr.ErrDegenerate)
			}
		}
		saddles[i] = s
	}
	out, err := cutSaddles(ctx, env, mesher, b, fp, saddles, d, faceIndex)
	if err != nil {
		return nil, 0, true, err
	}
	return out, 2, true, nil
}

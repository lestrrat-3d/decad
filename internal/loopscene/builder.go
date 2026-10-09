package loopscene

import (
	"context"
	"errors"
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	"github.com/lestrrat-3d/decad/internal/linkagebound"
	"github.com/lestrrat-3d/decad/internal/motionbound"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
)

// Link holds the loop facts needed to build a private sketch scene.
type Link struct {
	Index                       int
	Parent                      *Link
	PinPos, NextPos, RiderPos   r3.Vec
	Slide, Anchored, BesideRail bool
	Dir                         motionbound.RatVec
	Anchor                      [2]motionbound.RatVec
	AnchorLength                proofbound.RatInterval
}

// Bar is one certified distance between loop pins.
type Bar struct {
	Link      *Link
	Own, Next r3.Vec
	Down, Up  float64
}

// Loop is a read-only scene input from the root linkage record.
type Loop struct {
	Links, SideA, SideB []*Link
	Bars                []Bar
	Common, Slide       *Link
	CommonPins          [2]r3.Vec
	ClosureCenter       r3.Vec
	Normal              motionbound.RatVec
	Coord               int
	Reach               float64
	PrimaryDir          *r3.Vec
}

// Joint is a loop link's resolved joint.
type Joint struct {
	Link     *Link
	Revolute bool
	Axis     r3.Vec
}

// Input selects a driver and its dependent joints from the loop.
type Input struct {
	Loop                    *Loop
	Joints                  []Joint
	Driver                  int
	Deps                    []int
	Mirror, HalfTurn, Ahead bool
	Offset                  proofbound.RatInterval
}

// Plane holds the exact directions and their float frame.
type Plane struct {
	Frame      r3.Frame
	U, V       motionbound.RatVec
	ULen, VLen proofbound.RatInterval
}

func (pl Plane) coords(p r3.Vec) (proofbound.RatInterval, proofbound.RatInterval) {
	return pl.coordsRat(linkagebound.ExactVec(p))
}

func (pl Plane) coordsRat(p motionbound.RatVec) (proofbound.RatInterval, proofbound.RatInterval) {
	return linkagebound.PlaneCoordinates(p, pl.U, pl.V, pl.ULen, pl.VLen)
}

func (lp *Loop) sceneFrame(mirror, halfTurn bool) (Plane, error) {
	pl, err := linkagebound.SceneFrame(lp.Normal, lp.Coord, lp.PrimaryDir, mirror, halfTurn)
	if err != nil {
		return Plane{}, err
	}
	return Plane{Frame: pl.Frame, U: pl.U, V: pl.V, ULen: pl.ULen, VLen: pl.VLen}, nil
}

func (lp *Loop) anchored(k *Link) bool   { return k.Anchored }
func (lp *Loop) besideRail(k *Link) bool { return k.BesideRail }
func (lp *Loop) riderAt(k *Link) r3.Vec  { return k.RiderPos }
func (lp *Loop) linkNext(k *Link) r3.Vec { return k.NextPos }
func (lp *Loop) commonPins() (r3.Vec, r3.Vec) {
	return lp.CommonPins[0], lp.CommonPins[1]
}
func (lp *Loop) anchorAt(k *Link, ahead bool) motionbound.RatVec {
	if ahead {
		return k.Anchor[1]
	}
	return k.Anchor[0]
}
func (lp *Loop) anchorLength(k *Link) proofbound.RatInterval { return k.AnchorLength }

func (lp *Loop) planeSqRat(p, q motionbound.RatVec) *big.Rat {
	var d motionbound.RatVec
	for i := range 3 {
		d[i] = new(big.Rat).Sub(p[i], q[i])
	}
	along := linkagebound.Dot(d, lp.Normal)
	along.Mul(along, along)
	along.Quo(along, linkagebound.Dot(lp.Normal, lp.Normal))
	return along.Sub(linkagebound.Dot(d, d), along)
}

func (lp *Loop) scaleFor(dir motionbound.RatVec) *big.Rat {
	_, exp := math.Frexp(lp.Reach / proofbound.RatSqrtDown(linkagebound.Dot(dir, dir)))
	return new(big.Rat).SetFloat64(math.Ldexp(1, exp))
}

func (k *Link) pin() r3.Vec                          { return k.PinPos }
func isSlide(k *Link) bool                           { return k.Slide }
func slideDir(k *Link) motionbound.RatVec            { return k.Dir }
func ratVecExact(v r3.Vec) motionbound.RatVec        { return linkagebound.ExactVec(v) }
func ratVecFloat(v motionbound.RatVec) r3.Vec        { return linkagebound.VecFloat(v) }
func ratNeg(v motionbound.RatVec) motionbound.RatVec { return linkagebound.NegVec(v) }
func floatBox(iv proofbound.RatInterval) sketch.Interval {
	return sketch.Interval{Lo: proofbound.RatFloatDown(iv.Lo), Hi: proofbound.RatFloatUp(iv.Hi)}
}
func ratStep(p, dir motionbound.RatVec, lambda *big.Rat) motionbound.RatVec {
	var out motionbound.RatVec
	for i := range 3 {
		out[i] = new(big.Rat).Mul(lambda, dir[i])
		out[i].Add(out[i], p[i])
	}
	return out
}

// Pin is a scene point paired with its exact plane-position enclosure.
type Pin struct {
	P    *sketch.Point
	At   r3.Vec
	U, V proofbound.RatInterval
}

// Result contains the built sketch and the dimensions the enclosure chain reads.
type Result struct {
	Plane             Plane
	Sketch            *sketch.Sketch
	Driver            sketch.Dimension
	Driven            []sketch.Dimension
	Angular, Anchored []bool
	Signs             []int
	Options           []sketch.EncloseOption
	Pins              []Pin
	Offset            proofbound.RatInterval
}

// Build creates the private sketch scene and solves its zero-pose dimensions.
func Build(ctx context.Context, in Input) (*Result, error) {
	lp := in.Loop
	spec := struct{ joints []Joint }{in.Joints}
	ld := struct {
		driver int
		deps   []int
	}{in.Driver, in.Deps}
	flip := struct{ mirror, halfTurn, ahead bool }{in.Mirror, in.HalfTurn, in.Ahead}
	offset := in.Offset
	plane, err := lp.sceneFrame(flip.mirror, flip.halfTurn)
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's plane frame: %w`, decaderr.ErrNotFinite, err)
	}
	w := sketch.NewWorld()
	sk, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, fmt.Errorf(`%w: a loop's scene: %w`, decaderr.ErrUnsupported, err)
	}
	sc := &Result{Plane: plane, Sketch: sk, Offset: offset}
	// fix grounds p at the exact plane position x × y: a fixed box around it
	// where either coordinate is not one float, as on a tilted loop.
	fix := func(p *sketch.Point, x, y proofbound.RatInterval) {
		sk.Fix(p)
		bx, by := floatBox(x), floatBox(y)
		if bx.Lo != bx.Hi || by.Lo != by.Hi {
			sc.Options = append(sc.Options, sketch.WithFixedBox(p, bx, by))
		}
	}
	points := make(map[r3.Vec]int) // each pin's index in sc.Pins
	pinAt := func(at r3.Vec) Pin {
		if n, ok := points[at]; ok {
			return sc.Pins[n]
		}
		local := plane.Frame.ToLocal(at)
		x, y := plane.coords(at)
		pin := Pin{P: sk.CreatePoint(local.X, local.Y), At: at, U: x, V: y}
		points[at] = len(sc.Pins)
		sc.Pins = append(sc.Pins, pin)
		return pin
	}
	point := func(at r3.Vec) *sketch.Point { return pinAt(at).P }
	fixPin := func(at r3.Vec) {
		pin := pinAt(at)
		fix(pin.P, pin.U, pin.V)
	}
	fixed := func(at r3.Vec, du float64) *sketch.Point {
		local := plane.Frame.ToLocal(at)
		x, y := plane.coords(at)
		if du != 0 {
			local.X += du
			shift := proofarith.FloatRat(du)
			x = proofbound.IntervalOwned(new(big.Rat).Add(x.Lo, shift), new(big.Rat).Add(x.Hi, shift))
		}
		p := sk.CreatePoint(local.X, local.Y)
		fix(p, x, y)
		return p
	}
	// newPin is a free point of the scene at the exact zero-pose position at
	// that no other feature shares — an anchored rail's points, a rider
	// between two slides — listed for the falsifier like every pin.
	newPin := func(at motionbound.RatVec) *sketch.Point {
		f := ratVecFloat(at)
		local := plane.Frame.ToLocal(f)
		x, y := plane.coordsRat(at)
		pin := Pin{P: sk.CreatePoint(local.X, local.Y), At: f, U: x, V: y}
		sc.Pins = append(sc.Pins, pin)
		return pin.P
	}
	fixedAt := func(at motionbound.RatVec) *sketch.Point {
		local := plane.Frame.ToLocal(ratVecFloat(at))
		x, y := plane.coordsRat(at)
		p := sk.CreatePoint(local.X, local.Y)
		fix(p, x, y)
		return p
	}
	lines := make(map[*Link]*sketch.Line)
	var cons []sketch.Constraint
	// hold keeps p and q at the exact plane distance between pa and pb, stated
	// like a bar: its target the interval between the floats around the root.
	hold := func(p, q *sketch.Point, pa, pb motionbound.RatVec) error {
		sq := lp.planeSqRat(pa, pb)
		down, up := proofbound.RatSqrtDown(sq), proofbound.RatSqrtUp(sq)
		if proofbound.IsNonFinite(up) || !(down > 0) {
			return fmt.Errorf(`%w: a loop rail's distance is not representable`, decaderr.ErrNotFinite)
		}
		dist := sketch.NewDistance(p, q, up)
		cons = append(cons, dist)
		sc.Options = append(sc.Options, sketch.WithTargetRange(dist, down, up))
		return nil
	}
	riders := make(map[*Link]*sketch.Point)
	rider := func(k *Link) *sketch.Point {
		if p, ok := riders[k]; ok {
			return p
		}
		p := point(lp.riderAt(k))
		if lp.besideRail(k) {
			p = newPin(ratVecExact(lp.riderAt(k)))
		}
		riders[k] = p
		return p
	}
	var commonLine *sketch.Line
	var railStart, slider *sketch.Point
	if lp.Slide == nil {
		pa, pb := lp.commonPins()
		commonLine = sk.CreateLine(point(pa), point(pb))
		fixPin(pa)
		fixPin(pb)
	} else {
		// The primary slide's rail is the fixed line through its rider's
		// zero-pose position along u; the rider rides it. The rail is the
		// line Common's other pin, and the slide's children, measure their
		// angles from.
		pin := lp.riderAt(lp.Slide)
		slider = rider(lp.Slide)
		railStart = fixed(pin, 0)
		commonLine = sk.CreateLine(railStart, fixed(pin, 1))
		lines[lp.Slide] = commonLine
		cons = append(cons, sketch.NewPointOnLine(slider, commonLine))
		other := lp.SideA
		if len(lp.SideA) > 0 && lp.SideA[0] == lp.Slide {
			other = lp.SideB
		}
		switch {
		case len(other) == 0:
			fixPin(lp.ClosureCenter)
		case !isSlide(other[0]):
			fixPin(other[0].pin())
		}
	}
	for _, bar := range lp.Bars {
		if bar.Link == lp.Common {
			continue
		}
		own, next := point(bar.Own), point(bar.Next)
		lines[bar.Link] = sk.CreateLine(own, next)
		dist := sketch.NewDistance(own, next, bar.Up)
		cons = append(cons, dist)
		sc.Options = append(sc.Options, sketch.WithTargetRange(dist, bar.Down, bar.Up))
	}
	driverLink := spec.joints[ld.driver].Link
	anchors := make(map[*Link]*sketch.Point)
	lineAt := make(map[*Link][2]motionbound.RatVec) // a rail-held link's line ends at the zero pose
	for _, k := range lp.Links {
		if !lp.anchored(k) {
			continue
		}
		p0 := ratVecExact(lp.riderAt(k))
		at := lp.anchorAt(k, flip.ahead && k == driverLink)
		var a *sketch.Point
		var rail *sketch.Line
		par := k.Parent
		switch {
		case par == lp.Common:
			a = fixedAt(at)
			rail = sk.CreateLine(a, fixedAt(p0))
		case isSlide(par):
			// The parent's rider X lies on this rail at P₀; a second point Z
			// of the parent on its own rail, at the anchor's scale, fixes the
			// parent's turn.
			x := rider(par)
			zAt := ratStep(p0, slideDir(par), lp.scaleFor(slideDir(par)))
			z := newPin(zAt)
			a = newPin(at)
			cons = append(cons, sketch.NewPointOnLine(z, lines[par]))
			if err := errors.Join(hold(x, z, p0, zAt), hold(x, a, p0, at), hold(z, a, zAt, at)); err != nil {
				return nil, err
			}
			rail = sk.CreateLine(a, x)
		default:
			// The parent's frame is its pin O and a point C off the rail at
			// the anchor's scale. The rail's two points, A and B = 2P₀ − A
			// on the rider's other side, are each held to both, by
			// triangles that stay well shaped wherever O lies.
			o, oAt := point(par.pin()), ratVecExact(par.pin())
			perp := linkagebound.Cross(lp.Normal, slideDir(k))
			cAt := ratStep(oAt, perp, lp.scaleFor(perp))
			var bAt motionbound.RatVec
			for i := range 3 {
				bAt[i] = new(big.Rat).Sub(new(big.Rat).Add(p0[i], p0[i]), at[i])
			}
			c := newPin(cAt)
			a = newPin(at)
			b := newPin(bAt)
			if err := errors.Join(hold(o, c, oAt, cAt), hold(o, a, oAt, at), hold(c, a, cAt, at), hold(o, b, oAt, bAt), hold(c, b, cAt, bAt)); err != nil {
				return nil, err
			}
			rail = sk.CreateLine(a, b)
			lines[par], lineAt[par] = rail, [2]motionbound.RatVec{at, bAt}
		}
		anchors[k] = a
		lines[k] = rail
		cons = append(cons, sketch.NewPointOnLine(rider(k), rail))
	}
	parentLine := func(k *Link) *sketch.Line {
		if k.Parent == lp.Common {
			return commonLine
		}
		return lines[k.Parent]
	}
	n := lp.Normal
	if flip.mirror {
		n = ratNeg(n)
	}
	senseOf := func(k int) int {
		if spec.joints[k].Revolute {
			return linkagebound.Dot(ratVecExact(spec.joints[k].Axis), n).Sign()
		}
		if lp.anchored(spec.joints[k].Link) {
			return 1
		}
		return linkagebound.Dot(ratVecExact(spec.joints[k].Axis), plane.U).Sign()
	}
	mid, _ := linkagebound.Midpoint(offset).Float64()
	switch {
	case driverLink == lp.Slide:
		sc.Driver = sketch.NewHorizontalDistance(railStart, slider, 0)
	case lp.anchored(driverLink):
		sc.Driver = sketch.NewDistance(anchors[driverLink], rider(driverLink), mid)
	case driverLink.Parent == lp.Common && lp.besideRail(driverLink):
		ends := lineAt[driverLink]
		refLine := sk.CreateLine(fixedAt(ends[0]), fixedAt(ends[1]))
		sc.Driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	case driverLink.Parent == lp.Common:
		next := lp.linkNext(driverLink)
		refLine := sk.CreateLine(point(driverLink.pin()), fixed(next, 0))
		sc.Driver = sketch.NewAngle(refLine, lines[driverLink], 0)
	default:
		// Below Common the driver turns from its parent's line, offset r₀.
		sc.Driver = sketch.NewAngle(parentLine(driverLink), lines[driverLink], mid)
	}
	cons = append(cons, sc.Driver)
	for _, d := range ld.deps {
		link := spec.joints[d].Link
		var dim sketch.Dimension
		switch {
		case link == lp.Slide:
			h := sketch.NewHorizontalDistance(railStart, slider, 0)
			h.SetDriven(true)
			dim = h
		case lp.anchored(link):
			r, _ := linkagebound.Midpoint(lp.anchorLength(link)).Float64()
			dist := sketch.NewDistance(anchors[link], rider(link), r)
			dist.SetDriven(true)
			dim = dist
		default:
			a := sketch.NewAngle(parentLine(link), lines[link], 0)
			a.SetDriven(true)
			dim = a
		}
		sc.Driven = append(sc.Driven, dim)
		sc.Angular = append(sc.Angular, !isSlide(link))
		sc.Anchored = append(sc.Anchored, lp.anchored(link))
		sc.Signs = append(sc.Signs, senseOf(d))
		cons = append(cons, dim)
	}
	sk.AddConstraint(cons...)
	if _, err := sk.Solve(ctx); err != nil {
		return nil, fmt.Errorf(`%w: the loop's scene does not solve at the zero pose: %w`, decaderr.ErrUnsupported, err)
	}
	return sc, nil
}

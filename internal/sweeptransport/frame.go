// Package sweeptransport carries held endpoint frames beside rational
// intervals enclosing the exact rotation-minimizing transport of a Sweep path.
package sweeptransport

import (
	"fmt"
	"math"
	"math/big"

	"github.com/lestrrat-3d/decad/internal/decaderr"
	proofarith "github.com/lestrrat-3d/decad/internal/proof"
	"github.com/lestrrat-3d/decad/internal/proofbound"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/units"
)

// Frame holds a transported section frame and its exact interval enclosure.
type Frame struct {
	Frame r3.Frame

	OriginExact proofbound.IvVec3
	UExact      proofbound.IvVec3
	VExact      proofbound.IvVec3
	NExact      proofbound.IvVec3

	OriginBound float64
	UBound      float64
	VBound      float64
	NBound      float64
}

func InitialFrame(frame r3.Frame) (Frame, error) {
	origin, okO := proofbound.IvVec3Of(frame.Origin())
	u, okU := proofbound.IvVec3Of(frame.U())
	v, okV := proofbound.IvVec3Of(frame.V())
	n, okN := proofbound.IvVec3Of(frame.N())
	if !okO || !okU || !okV || !okN {
		return Frame{}, fmt.Errorf(`%w: the source sweep frame is not finite`, decaderr.ErrUnsupported)
	}
	return newSweepTransportFrame(frame, origin, u, v, n)
}

// Line transports a frame along one recorded straight span.
func Line(current Frame, start, end r3.Vec) (Frame, error) {
	deltaExact := proofbound.IvVec3Sub(mustIVVec3Of(end), mustIVVec3Of(start))
	originExact := proofbound.IvVec3Add(current.OriginExact, deltaExact)
	if exact, ok := exactSweepTransportFrame(
		originExact,
		current.UExact,
		current.VExact,
		current.NExact,
	); ok {
		return exact, nil
	}

	delta := end.Sub(start)
	if !proofbound.FiniteVec(delta) {
		return Frame{}, fmt.Errorf(`%w: a line span's translation is outside the representable range`, decaderr.ErrUnsupported)
	}
	origin := current.Frame.Origin().Add(delta)
	if !proofbound.FiniteVec(origin) {
		return Frame{}, fmt.Errorf(`%w: a line span's transported origin is outside the representable range`, decaderr.ErrUnsupported)
	}
	frame, err := r3.NewFrame(origin, current.Frame.U(), current.Frame.V())
	if err != nil {
		return Frame{}, fmt.Errorf(`%w: a line span produced no finite transported frame: %s`, decaderr.ErrUnsupported, err)
	}
	return newSweepTransportFrame(
		frame,
		originExact,
		current.UExact,
		current.VExact,
		current.NExact,
	)
}

// Arc holds the certified geometry of one recorded circular span.
type Arc struct {
	CenterExact, AxisExact proofbound.IvVec3
	Sin, Cos               proofbound.RatInterval
	Phi                    float64
}

// TransportArc carries a frame through the span's certified rotation.
func TransportArc(current Frame, arc Arc) (Frame, error) {
	centerExact, axisExact := arc.CenterExact, arc.AxisExact
	sin, cos := arc.Sin, arc.Cos

	rotateDirection := func(vector proofbound.IvVec3) proofbound.IvVec3 {
		oneMinusCos := proofbound.IntervalSub(proofbound.PointInterval(big.NewRat(1, 1)), cos)
		return proofbound.IvVec3Add(
			proofbound.IvVec3Mul(vector, cos),
			proofbound.IvVec3Add(
				proofbound.IvVec3Mul(proofbound.IvVec3Cross(axisExact, vector), sin),
				proofbound.IvVec3Mul(axisExact, proofbound.IntervalMul(oneMinusCos, proofbound.IvVec3Dot(axisExact, vector))),
			),
		)
	}
	rotatePoint := func(point proofbound.IvVec3) proofbound.IvVec3 {
		return proofbound.IvVec3Add(centerExact, rotateDirection(proofbound.IvVec3Sub(point, centerExact)))
	}

	originExact := rotatePoint(current.OriginExact)
	uExact := rotateDirection(current.UExact)
	vExact := rotateDirection(current.VExact)
	nExact := rotateDirection(current.NExact)
	if exact, ok := exactSweepTransportFrame(originExact, uExact, vExact, nExact); ok {
		return exact, nil
	}

	center, ok := heldSweepRatVec(arc.CenterExact)
	if !ok {
		return Frame{}, fmt.Errorf(`%w: an arc span's center is outside the representable range`, decaderr.ErrUnsupported)
	}
	axis, ok := heldSweepUnitVector(arc.AxisExact)
	if !ok {
		return Frame{}, fmt.Errorf(`%w: an arc span's axis is outside the representable range`, decaderr.ErrUnsupported)
	}
	rotation, err := r3.RotationAround(center, axis, units.Radians(arc.Phi))
	if err != nil {
		return Frame{}, fmt.Errorf(`%w: an arc span produced no finite rotation: %s`, decaderr.ErrUnsupported, err)
	}
	origin := rotation.Apply(current.Frame.Origin())
	u := rotation.ApplyDir(current.Frame.U())
	v := rotation.ApplyDir(current.Frame.V())
	if !proofbound.FiniteVec(origin) || !proofbound.FiniteVec(u) || !proofbound.FiniteVec(v) {
		return Frame{}, fmt.Errorf(`%w: an arc span produced a non-finite transported frame`, decaderr.ErrUnsupported)
	}
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		return Frame{}, fmt.Errorf(`%w: an arc span produced no transported frame: %s`, decaderr.ErrUnsupported, err)
	}
	return newSweepTransportFrame(frame, originExact, uExact, vExact, nExact)
}

// exactSweepTransportFrame keeps quadrantal coordinate-axis transport exact.
// The rational rotation path above collapses to point intervals for these
// spans, so publishing those points directly avoids introducing libm's small
// nonzero cosine at a quarter turn. It is deliberately a strict fast path:
// any rounded coordinate or any normalization movement falls back to the
// bounded general transport.
func exactSweepTransportFrame(
	originExact, uExact, vExact, nExact proofbound.IvVec3,
) (Frame, bool) {
	origin, okO := exactSweepIVVec(originExact)
	u, okU := exactSweepIVVec(uExact)
	v, okV := exactSweepIVVec(vExact)
	if !okO || !okU || !okV {
		return Frame{}, false
	}
	frame, err := r3.NewFrame(origin, u, v)
	if err != nil {
		return Frame{}, false
	}
	out, err := newSweepTransportFrame(frame, originExact, uExact, vExact, nExact)
	if err != nil || out.OriginBound != 0 || out.UBound != 0 || out.VBound != 0 || out.NBound != 0 {
		return Frame{}, false
	}
	return out, true
}

func exactSweepIVVec(vector proofbound.IvVec3) (r3.Vec, bool) {
	components := [3]float64{}
	for i, coordinate := range vector {
		if coordinate.Lo.Cmp(coordinate.Hi) != 0 {
			return r3.Vec{}, false
		}
		held, exact := coordinate.Lo.Float64()
		if !exact || math.IsNaN(held) || math.IsInf(held, 0) {
			return r3.Vec{}, false
		}
		components[i] = held
	}
	return r3.NewVec(components[0], components[1], components[2]), true
}

func newSweepTransportFrame(
	frame r3.Frame,
	originExact, uExact, vExact, nExact proofbound.IvVec3,
) (Frame, error) {
	originBound, okO := sweepIVVecError(originExact, frame.Origin())
	uBound, okU := sweepIVVecError(uExact, frame.U())
	vBound, okV := sweepIVVecError(vExact, frame.V())
	nBound, okN := sweepIVVecError(nExact, frame.N())
	if !okO || !okU || !okV || !okN {
		return Frame{}, fmt.Errorf(`%w: a transported frame has no finite publication bound`, decaderr.ErrUnsupported)
	}
	return Frame{
		Frame: frame,

		OriginExact: originExact,
		UExact:      uExact,
		VExact:      vExact,
		NExact:      nExact,

		OriginBound: originBound,
		UBound:      uBound,
		VBound:      vBound,
		NBound:      nBound,
	}, nil
}

func sweepIVVecError(exact proofbound.IvVec3, held r3.Vec) (float64, bool) {
	ex := proofbound.IntervalFloatError(exact[0], held.X)
	ey := proofbound.IntervalFloatError(exact[1], held.Y)
	ez := proofbound.IntervalFloatError(exact[2], held.Z)
	if !finiteValues(ex, ey, ez) {
		return 0, false
	}
	squared := proofbound.AbsSumUpper(proofbound.ProductUpper(ex, ex), proofbound.ProductUpper(ey, ey), proofbound.ProductUpper(ez, ez))
	bound := proofbound.UpRound(math.Sqrt(squared))
	return bound, !math.IsNaN(bound) && !math.IsInf(bound, 0)
}

func mustIVVec3Of(vector r3.Vec) proofbound.IvVec3 {
	out, ok := proofbound.IvVec3Of(vector)
	if !ok {
		panic("decad: a recorded Sweep path point must be finite")
	}
	return out
}

func finiteValues(values ...float64) bool {
	for _, value := range values {
		if math.IsNaN(value) || math.IsInf(value, 0) {
			return false
		}
	}
	return true
}

func heldSweepRatVec(vector proofbound.IvVec3) (r3.Vec, bool) {
	var held [3]float64
	for i, component := range vector {
		value := component.Lo
		held[i], _ = value.Float64()
		if !finiteValues(held[i], proofarith.RationalFloatError(value, held[i])) {
			return r3.Vec{}, false
		}
	}
	return r3.NewVec(held[0], held[1], held[2]), true
}

func heldSweepUnitVector(unit proofbound.IvVec3) (r3.Vec, bool) {
	component := func(value proofbound.RatInterval) float64 {
		midpoint := new(big.Rat).Add(value.Lo, value.Hi)
		midpoint.Quo(midpoint, big.NewRat(2, 1))
		held, _ := midpoint.Float64()
		return held
	}
	held := r3.NewVec(component(unit[0]), component(unit[1]), component(unit[2]))
	if !proofbound.FiniteVec(held) {
		return r3.Vec{}, false
	}
	return held.Normalize()
}

package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/r3"
)

// MirrorPlane is what a body may be mirrored across: a [MirrorFrame] the
// caller builds, or a [MirrorFace] naming a planar face of a body. The set is
// sealed (docs/mirror-pattern-design.md §4.1).
type MirrorPlane interface{ mirrorPlane() }

// MirrorFrame mirrors across the plane through Frame's origin spanned by its
// U and V axes, the plane r3.Reflection takes. The zero Frame names no plane
// and is ErrDegenerate at the call.
type MirrorFrame struct {
	Frame r3.Frame
}

// MirrorFace mirrors across a planar face of Body, selected and never pointed
// at (core §9). Body is what Face resolves against: it MUST be a live body of
// the same document as the receiver (ErrForeignBody, ErrRetiredBody), and the
// receiver itself is the ordinary choice. Resolving the plane does not consume
// Body.
//
// Face resolves through SelectFaces(Body) under the implicit exactly-one rule:
// zero or several faces is ErrCardinality with Expected "exactly 1", and a
// failed assertion of the selector's own is returned unchanged. A curved face
// is ErrDegenerate. A flat face with no analytic [Plane] surface, such as a
// mesh-boolean [Faceted] face, states no exact plane and is ErrUnsupported.
// The mirror plane is the face's own Plane.Frame. Under [WithJoin], Body must
// be the receiver and Face may name several walls on one line: zero faces is
// ErrCardinality with Expected "at least 1".
type MirrorFace struct {
	Body *Body
	Face FaceSelector
}

// The sealed set.
func (MirrorFrame) mirrorPlane() {}
func (MirrorFace) mirrorPlane()  {}

// MirrorOption configures [Body.Mirrored]. [WithJoin] is the one option.
type MirrorOption interface {
	mirrorOption()
}

// errNilMirrorPlane rejects a nil plane, or a nil variant pointer: either
// names no plane to mirror across.
var errNilMirrorPlane = fmt.Errorf(`%w: nil mirror plane`, ErrDegenerate)

// Mirrored returns a new body carrying the receiver's mirror image across
// plane, retiring the receiver. It is [Body.Placed] under r3.Reflection of the
// resolved plane (docs/mirror-pattern-design.md §4.2): the reflection is
// carried in the body's accumulated placement, and every reading — volume,
// centroid, outward normals, tessellation winding, export — follows it.
//
// The gates are Placed's — a nil context is ErrDegenerate, a body this
// evaluator did not build is ErrUnsupported, and a canceled context stops the
// rebuild before the document changes — plus the plane's own resolution (see
// [MirrorFrame] and [MirrorFace]). A nil plane or a nil option is
// ErrDegenerate. With [WithJoin] it returns the union of the receiver and its
// image instead, on the class WithJoin states. A refused call leaves the
// document unchanged.
func (b *Body) Mirrored(ctx context.Context, plane MirrorPlane, opts ...MirrorOption) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mirror`, ErrDegenerate)
	}
	join := false
	for _, o := range opts {
		if o == nil {
			return nil, fmt.Errorf(`%w: a nil option names nothing to apply`, ErrDegenerate)
		}
		// MirrorOption carries one variant, so this is an if rather than a
		// single-case type switch.
		if _, ok := o.(mirrorJoinOption); ok {
			join = true
		}
	}
	if join {
		return b.mirroredJoin(ctx, plane)
	}
	t, err := b.mirrorTransform(plane)
	if err != nil {
		return nil, err
	}
	return b.Placed(ctx, t)
}

// MirroredCopy returns a new live body carrying the receiver's mirror image
// across plane, leaving the receiver LIVE. It is [Body.PlacedCopy] under
// r3.Reflection of the resolved plane, with the gates [Body.Mirrored] states.
func (b *Body) MirroredCopy(ctx context.Context, plane MirrorPlane) (*Body, error) {
	if ctx == nil {
		return nil, fmt.Errorf(`%w: a nil context cannot control a mirror`, ErrDegenerate)
	}
	t, err := b.mirrorTransform(plane)
	if err != nil {
		return nil, err
	}
	return b.PlacedCopy(ctx, t)
}

// mirrorTransform runs the receiver's identity gates, then resolves plane to
// the reflection across it. The receiver gates come first so a call on a
// retired or foreign body reports that, whatever plane it names.
func (b *Body) mirrorTransform(plane MirrorPlane) (r3.Transform, error) {
	if b == nil || b.doc == nil {
		return r3.Transform{}, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := b.doc
	if err := d.requireLive(b); err != nil {
		return r3.Transform{}, err
	}
	frame, err := d.resolveMirrorPlane(plane)
	if err != nil {
		return r3.Transform{}, err
	}
	t, err := r3.Reflection(frame)
	switch {
	case err == nil:
		return t, nil
	case errors.Is(err, r3.ErrNonFinite):
		return r3.Transform{}, fmt.Errorf(`%w: the mirror plane lies too far from the origin to reflect across: %s`, ErrNotFinite, err)
	default:
		return r3.Transform{}, fmt.Errorf(`%w: the mirror frame names no plane: %s`, ErrDegenerate, err)
	}
}

// resolveMirrorPlane returns the frame whose UV plane is the mirror plane. The
// variants seal with value receivers, so a pointer to either is accepted and
// a nil pointer is rejected like a nil plane.
func (d *Document) resolveMirrorPlane(plane MirrorPlane) (r3.Frame, error) {
	switch p := plane.(type) {
	case MirrorFrame:
		return p.Frame, nil
	case *MirrorFrame:
		if p == nil {
			return r3.Frame{}, errNilMirrorPlane
		}
		return p.Frame, nil
	case MirrorFace:
		return d.resolveMirrorFace(p)
	case *MirrorFace:
		if p == nil {
			return r3.Frame{}, errNilMirrorPlane
		}
		return d.resolveMirrorFace(*p)
	default:
		// The interface is sealed, so the only value left is nil.
		return r3.Frame{}, errNilMirrorPlane
	}
}

// resolveMirrorFace runs MirrorFace's gates and returns the selected face's
// own plane frame.
func (d *Document) resolveMirrorFace(mf MirrorFace) (r3.Frame, error) {
	if mf.Body == nil {
		return r3.Frame{}, fmt.Errorf(`%w: a mirror face names no body to resolve against`, ErrDegenerate)
	}
	if err := d.requireLive(mf.Body); err != nil {
		return r3.Frame{}, err
	}
	face, err := selectImpliedOneFace(mf.Body, mf.Face, "the mirror face")
	if err != nil {
		return r3.Frame{}, err
	}
	pl, ok := face.Surface().(Plane)
	switch {
	case ok:
		return pl.Frame, nil
	case face.isPlanar():
		return r3.Frame{}, fmt.Errorf(`%w: the mirror face is flat but its surface is %T, which states no exact plane to mirror across; name a face with an analytic Plane surface, or use a MirrorFrame`, ErrUnsupported, face.Surface())
	default:
		return r3.Frame{}, fmt.Errorf(`%w: the mirror face is not planar (its surface is %T)`, ErrDegenerate, face.Surface())
	}
}

package decad

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad/internal/mirrorjoin"
	"github.com/lestrrat-3d/decad/internal/opcommand"
	"github.com/lestrrat-3d/decad/internal/surfacegeom"
	"github.com/lestrrat-3d/r3"
)

// MirrorPlane is what a body may be mirrored across: a [MirrorFrame] the
// caller builds, or a [MirrorFace] naming a planar face of a body. The set is
// sealed (agent-docs/mirror-pattern-design.md §4.1).
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
// The mirror plane is the face's own Plane.Frame. Under [MirroredJoin], Body must
// be the receiver and Face may name several walls on one line: zero faces is
// ErrCardinality with Expected "at least 1".
type MirrorFace struct {
	Body *Body
	Face FaceSelector
}

// The sealed set.
func (MirrorFrame) mirrorPlane() {}
func (MirrorFace) mirrorPlane()  {}

// errNilMirrorPlane rejects a nil plane, or a nil variant pointer: either
// names no plane to mirror across.
var errNilMirrorPlane = fmt.Errorf(`%w: nil mirror plane`, ErrDegenerate)

// copyMirrorPlane copies the plane at the call that stores it
// (agent-docs/command-api-design.md rule A3): the face selector of a
// [MirrorFace], and the pointee of a non-nil pointer to either variant. A
// selector that is no decad query is stored as given.
func copyMirrorPlane(plane MirrorPlane) MirrorPlane {
	switch p := plane.(type) {
	case MirrorFace:
		p.Face = copyFaceSelector(p.Face)
		return p
	case *MirrorFace:
		if p == nil {
			return p
		}
		cp := *p
		cp.Face = copyFaceSelector(cp.Face)
		return &cp
	case *MirrorFrame:
		if p == nil {
			return p
		}
		cp := *p
		return &cp
	}
	return plane
}

// Mirrored returns the command that mirrors b across plane. Do returns a new
// body carrying b's mirror image across plane, retiring b. It calls
// [Transform] under r3.Reflection of the resolved plane
// (agent-docs/mirror-pattern-design.md §4.2): the reflection is carried in the
// body's accumulated placement, and every reading — volume, centroid, outward
// normals, tessellation winding, export — follows it.
//
// The gates are Transform's — a nil context is ErrDegenerate, a body this
// evaluator did not build is ErrUnsupported, and a canceled context stops the
// rebuild before the document changes — plus the plane's own resolution (see
// [MirrorFrame] and [MirrorFace]). A nil plane is ErrDegenerate, as is a nil b.
// To mirror and join in one body, see [MirroredJoin]. A refused call leaves
// the document unchanged. Mirrored copies plane when it is called, so a later
// change to a selector the caller kept does not reach the command.
func Mirrored(b *Body, plane MirrorPlane) *MirroredCommand {
	c := &MirroredCommand{b: b, plane: copyMirrorPlane(plane)}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// MirroredCommand mirrors a body across a plane. Build it with [Mirrored].
type MirroredCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	plane MirrorPlane     // copied at Mirrored
}

// Do mirrors the body and retires it, or returns the error of the first gate
// that refuses, leaving the body live. A nil ctx is ErrDegenerate, and a
// second call returns ErrConsumed.
func (c *MirroredCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationMirrored, c.run)
}

func (c *MirroredCommand) run(ctx context.Context) (*Body, error) {
	return mirrored(ctx, c.b, c.plane)
}

// mirrored is Mirrored's implementation. ctx is not nil, and the errors it
// returns carry no operation tag (agent-docs/command-api-design.md §11).
func mirrored(ctx context.Context, b *Body, plane MirrorPlane) (*Body, error) {
	t, err := b.mirrorTransform(plane)
	if err != nil {
		return nil, err
	}
	return transform(ctx, b, t, OperationMirrored, mirrorPlaneInputs(plane)...)
}

// MirroredCopy returns the command that copies b mirrored across plane. Do
// returns a new live body carrying b's mirror image across plane, leaving b
// LIVE. It copies under r3.Reflection of the resolved plane, with the gates
// [Mirrored] states. A nil b is ErrDegenerate.
func MirroredCopy(b *Body, plane MirrorPlane) *MirroredCopyCommand {
	c := &MirroredCopyCommand{b: b, plane: copyMirrorPlane(plane)}
	if b == nil {
		c.state.Record("body", errNilBody)
	}
	return c
}

// MirroredCopyCommand copies a body mirrored across a plane. Build it with
// [MirroredCopy].
type MirroredCopyCommand struct {
	state opcommand.State // consumed flag and first recorded fault
	b     *Body           // stored as given
	plane MirrorPlane     // copied at MirroredCopy
}

// Do copies the body mirrored and leaves it live, or returns the error of the
// first gate that refuses. A nil ctx is ErrDegenerate, and a second call
// returns ErrConsumed.
func (c *MirroredCopyCommand) Do(ctx context.Context) (*Body, error) {
	return runCommand(ctx, &c.state, OperationMirroredCopy, c.run)
}

func (c *MirroredCopyCommand) run(ctx context.Context) (*Body, error) {
	t, err := c.b.mirrorTransform(c.plane)
	if err != nil {
		return nil, err
	}
	return c.b.copyUnder(ctx, t, nil, OperationMirroredCopy, mirrorPlaneInputs(c.plane)...)
}

func mirrorPlaneInputs(plane MirrorPlane) []OperationInput {
	switch plane := plane.(type) {
	case MirrorFace:
		return []OperationInput{{Body: plane.Body, Role: "plane", Use: InputReferenced}}
	case *MirrorFace:
		if plane != nil {
			return []OperationInput{{Body: plane.Body, Role: "plane", Use: InputReferenced}}
		}
	}
	return nil
}

// mirrorTransform runs the receiver's identity gates, then resolves plane to
// the reflection across it. The receiver gates come first so a call on a
// retired or foreign body reports that, whatever plane it names.
func (b *Body) mirrorTransform(plane MirrorPlane) (r3.Transform, error) {
	if b == nil || b.n.Doc == nil {
		return r3.Transform{}, fmt.Errorf(`%w: the body belongs to no document`, ErrDegenerate)
	}
	d := documentHandle(b.n.Doc)
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
	pl, ok := face.n.Surface.(surfacegeom.Plane)
	// A retained tag with a stated normal bound is a stand-in for the
	// operand's plane, not an exact mirror plane
	// (agent-docs/faceted-source-retention-design.md §4).
	ok = ok && (!face.n.RetainedTag || face.n.NormalBound == 0)
	// A plane that states AxialDelta lies that far off its tag, so the image
	// across the tag is up to twice that off the true reflection, and the
	// placement has no channel to charge it (mirrorjoin.PlaneDeltaAdmitted).
	if ok && face.n.HasAxialDelta && !mirrorjoin.PlaneDeltaAdmitted(face.n.AxialDelta, pl.Frame.Origin()) {
		return r3.Frame{}, fmt.Errorf(`%w: the mirror face states a plane displaced from its tag by %g, which a reflection cannot carry; use a MirrorFrame`, ErrUnsupported, face.n.AxialDelta)
	}
	switch {
	case ok:
		return pl.Frame, nil
	case face.isPlanar():
		return r3.Frame{}, fmt.Errorf(`%w: the mirror face is flat but its surface is %T, which states no exact plane to mirror across; name a face with an analytic Plane surface, or use a MirrorFrame`, ErrUnsupported, face.Surface())
	default:
		return r3.Frame{}, fmt.Errorf(`%w: the mirror face is not planar (its surface is %T)`, ErrDegenerate, face.Surface())
	}
}

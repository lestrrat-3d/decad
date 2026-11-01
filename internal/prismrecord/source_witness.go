package prismrecord

import (
	"context"
	"slices"

	"github.com/lestrrat-3d/decad/internal/sectionrecord"
	"github.com/lestrrat-3d/decad/internal/survey2d"
	"github.com/lestrrat-3d/decad/internal/topo"
	"github.com/lestrrat-3d/r3"
)

type prismWitnessKey struct{}

// WithPrismSourceCapture requests the directly built source faces for a private operation capture.
func WithPrismSourceCapture(ctx context.Context, capture func(*PrismSourceWitness)) context.Context {
	return context.WithValue(ctx, prismWitnessKey{}, capture)
}

// PrismSourceWitness records direct builder faces for the admitted whole source segments.
type PrismSourceWitness struct {
	Body             *topo.Body
	Walls            []*topo.Face
	CapStart, CapEnd *topo.Face
	Segments         []sectionrecord.CurveSegment
	Frame            r3.Frame
	Xform            r3.Transform
	Z0, Z1           float64
	valid            bool
}

func newPrismSourceWitness(pp Payload) *PrismSourceWitness {
	if pp.SurfaceResult || pp.SectionDelta != 0 || pp.Z0Delta != 0 || pp.Z1Delta != 0 || len(pp.Profile.Holes) != 0 {
		return nil
	}
	segments := pp.Profile.Outer.Segments
	switch len(segments) {
	case 1:
		c, ok := segments[0].(sectionrecord.CircleSeg)
		if !ok || c.TStart != 0 || c.TEnd != 1 {
			return nil
		}
	case 4:
		for _, segment := range segments {
			line, ok := segment.(sectionrecord.LineSeg)
			if !ok || line.TStart != 0 || line.TEnd != 1 {
				return nil
			}
		}
	default:
		return nil
	}
	return &PrismSourceWitness{Segments: slices.Clone(segments), Frame: pp.Frame, Xform: pp.Xform, Z0: pp.Z0, Z1: pp.Z1}
}

func (w *PrismSourceWitness) captureWalks(walks []survey2d.SideWalk, faces []*topo.Face) {
	if w == nil || len(walks) != len(w.Segments) || len(faces) != len(walks) {
		return
	}
	for i, walk := range walks {
		if len(walk.Segs) != 1 || walk.Segs[0] != i || (len(walks) == 1 && !walk.Closed) {
			return
		}
	}
	w.Walls = slices.Clone(faces)
	w.valid = true
}

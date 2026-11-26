package apitest_test

import (
	"context"
	"errors"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// rsFeature is one feature that builds private tool bodies: target makes the body the feature works on and apply
// runs the feature on it. Both take the ctx the build runs under, so the same feature runs in a step, in a plain
// serial build, and on a body built before a run. readsOnly marks a feature that reads its target and never
// consumes it, so a pre-run target is not refused.
type rsFeature struct {
	name      string
	readsOnly bool
	target    func(ctx context.Context, doc *decad.Document) (*decad.Body, error)
	apply     func(ctx context.Context, doc *decad.Document, target *decad.Body) error
}

func rsMeshBlock(ctx context.Context, doc *decad.Document, lo, hi r3.Vec) (*decad.Body, error) {
	v, tris := cubeMesh(lo, hi)
	m, err := decad.NewMeshBody(ctx, v, tris)
	if err != nil {
		return nil, err
	}
	return decad.SolidFromMesh(doc, m).Do(ctx)
}

// rsRod is a cylinder of radius 5 and height 20 on the XY plane.
func rsRod(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		return nil, err
	}
	center := s.CreatePoint(0, 0)
	s.Fix(center)
	s.CreateCircle(center, 5)
	if _, err := s.Solve(ctx); err != nil {
		return nil, err
	}
	return decad.Extrude(doc, s, s.Profiles()[0], rsHeight(20)).Do(ctx)
}

// rsRibPart is a 40 x 40 x 20 block with a 30 x 30 pocket cut 15 deep from its top.
func rsRibPart(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	block, err := rsBlock(ctx, doc, 0, 0, 40, 40, 0, rsHeight(20))
	if err != nil {
		return nil, err
	}
	pocket, err := rsBlock(ctx, doc, 5, 5, 35, 35, 5, rsHeight(15))
	if err != nil {
		return nil, err
	}
	return decad.Cut(block, pocket).Do(ctx)
}

// rsRampStop is a 20 x 10 mesh block whose top slopes from 4 mm at x = 0 to 14 mm at x = 20.
func rsRampStop(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
	verts := []r3.Vec{
		r3.NewVec(0, 0, 0), r3.NewVec(20, 0, 0), r3.NewVec(20, 10, 0), r3.NewVec(0, 10, 0),
		r3.NewVec(0, 0, 4), r3.NewVec(20, 0, 14), r3.NewVec(20, 10, 14), r3.NewVec(0, 10, 4),
	}
	var tris [][3]int
	for _, q := range [][4]int{{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {3, 7, 6, 2}, {0, 4, 7, 3}, {1, 2, 6, 5}} {
		tris = append(tris, [3]int{q[0], q[1], q[2]}, [3]int{q[0], q[2], q[3]})
	}
	m, err := decad.NewMeshBody(ctx, verts, tris)
	if err != nil {
		return nil, err
	}
	return decad.SolidFromMesh(doc, m).Do(ctx)
}

// rsFeatures lists the features that build and remove private tool bodies.
func rsFeatures() []rsFeature {
	thread := func(leadIn bool) func(context.Context, *decad.Document, *decad.Body) error {
		return func(ctx context.Context, _ *decad.Document, rod *decad.Body) error {
			c := decad.Thread(rod, decad.Faces(decad.Cylindrical()), units.Millimeters(1.5))
			if leadIn {
				c = c.LeadIn(units.Millimeters(0.8))
			}
			_, err := c.Do(ctx)
			return err
		}
	}
	return []rsFeature{
		{name: "rib", target: rsRibPart, apply: func(ctx context.Context, _ *decad.Document, part *decad.Body) error {
			w := sketch.NewWorld()
			plane, err := w.CreateOffsetPlane(w.XY(), 4)
			if err != nil {
				return err
			}
			s, err := w.CreateSketch(plane)
			if err != nil {
				return err
			}
			a, b := s.CreatePoint(3, 20), s.CreatePoint(37, 20)
			s.Fix(a)
			s.Fix(b)
			line := s.CreateLine(a, b)
			if _, err := s.Solve(ctx); err != nil {
				return err
			}
			_, err = decad.WebPath(part, s, []sketch.Entity{line}, units.Millimeters(2), rsHeight(11)).Do(ctx)
			return err
		}},
		{name: "thread", target: rsRod, apply: thread(false)},
		{name: "thread-lead-in", target: rsRod, apply: thread(true)},
		{name: "emboss", target: func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return rsBlock(ctx, doc, 0, 0, 40, 30, 0, rsHeight(10))
		}, apply: func(ctx context.Context, _ *decad.Document, plate *decad.Body) error {
			w := sketch.NewWorld()
			plane, err := w.CreateOffsetPlane(w.XY(), 10)
			if err != nil {
				return err
			}
			s, err := w.CreateSketch(plane)
			if err != nil {
				return err
			}
			r := s.CreateRectangle(10, 10, 20, 16)
			s.Fix(r.A)
			if _, err := s.Solve(ctx); err != nil {
				return err
			}
			top := decad.Faces(decad.FaceCreatedBy(decad.CapEnd(plate)))
			_, err = decad.Emboss(plate, s, s.Profiles(), top, decad.EmbossRaised, units.Millimeters(2)).Do(ctx)
			return err
		}},
		{name: "wedge-chamfer", target: func(ctx context.Context, doc *decad.Document) (*decad.Body, error) {
			return rsMeshBlock(ctx, doc, r3.NewVec(0, 0, 0), r3.NewVec(20, 20, 10))
		}, apply: func(ctx context.Context, _ *decad.Document, b *decad.Body) error {
			sel := decad.Edges(decad.Convex(), decad.LongerThan(units.Millimeters(15)))
			_, err := decad.Chamfer(b, sel, units.Millimeters(1)).Do(ctx)
			return err
		}},
		{name: "tilted-extrude", readsOnly: true, target: rsRampStop,
			apply: func(ctx context.Context, doc *decad.Document, stop *decad.Body) error {
				var face decad.FaceSelector
				for _, f := range stop.Faces() {
					c, err := f.Centroid()
					if err != nil {
						return err
					}
					if c.Value.Sub(r3.NewVec(10, 5, 9)).Len() < 1e-9 {
						face = decad.Faces(decad.FaceCreatedBy(f.Origins()[0]))
					}
				}
				if face == nil {
					return errors.New("the ramp has no sloped face")
				}
				w := sketch.NewWorld()
				s, err := w.CreateSketch(w.XY())
				if err != nil {
					return err
				}
				r := s.CreateRectangle(5, 2, 15, 8)
				s.Fix(r.A)
				if _, err := s.Solve(ctx); err != nil {
					return err
				}
				_, err = decad.Extrude(doc, s, s.Profiles()[0], decad.ToFace{Body: stop, Face: face}).Do(ctx)
				return err
			}},
	}
}

package main

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/decad/internal/pointquery"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// threadRender shows an eight-turn V-groove opening through one end of a cylinder.
// Rendering this image in CI also exercises the consumer's full Coil and Cut.
func threadRender() imageRender {
	return imageRender{
		rel:      "features/thread.png",
		settings: solidlens.Settings{Width: featureWidth, Height: featureHeight},
		chord:    units.Millimeters(featureChordTolerance),
		scene: func(ctx context.Context, chord units.Value) (solidlens.Scene, error) {
			body, err := externalThread(ctx)
			if err != nil {
				return solidlens.Scene{}, err
			}
			models, err := oneModel(ctx, body, solidlens.RGB(0.32, 0.55, 0.68), chord)
			if err != nil {
				return solidlens.Scene{}, err
			}
			scene := featureScene(models...)
			scene.Camera = solidlens.Camera{
				Position: solidlens.Vec{X: 40, Y: 7},
				Target:   solidlens.Vec{Y: 7},
				Up:       solidlens.Vec{Z: 1},
				FOV:      27,
			}
			// Opposing axial light components illuminate both V-groove flanks.
			scene.DirectionalLights = []solidlens.DirectionalLight{
				{Direction: solidlens.Vec{X: -0.6, Y: -0.8, Z: 0.8}, Color: solidlens.RGB(1, 1, 1), Intensity: 0.8},
				{Direction: solidlens.Vec{X: -0.4, Y: 0.8, Z: 0.8}, Color: solidlens.RGB(0.7, 0.8, 1), Intensity: 0.45},
			}
			scene.PointLights = nil
			return scene, nil
		},
	}
}

func externalThread(ctx context.Context) (*decad.Body, error) {
	w := sketch.NewWorld()
	axis := decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}
	section, profile, err := sketchLoops(ctx, w, w.XY(), rectangle(0, 0, 5, 14))
	if err != nil {
		return nil, fmt.Errorf("draw thread cylinder: %w", err)
	}
	doc := decad.New()
	cylinder, err := decad.Revolve(ctx, doc, section, profile, axis, decad.FullRevolution{})
	if err != nil {
		return nil, fmt.Errorf("revolve thread cylinder: %w", err)
	}
	// Start a quarter pitch below the end so the thread runs out through the end face.
	// The blank trims the cutter; its other end leaves a short plain collar.
	const start = -0.375
	const halfWidth = 0.6928203230275509
	groove, grooveProfile, err := sketchLoops(ctx, w, w.XY(), []point{
		{4.1, start}, {5.3, start - halfWidth}, {5.3, start + halfWidth},
	})
	if err != nil {
		return nil, fmt.Errorf("draw thread groove: %w", err)
	}
	tool, err := decad.Coil(ctx, doc, groove, grooveProfile, axis, units.Millimeters(1.5), units.Scalar(8))
	if err != nil {
		return nil, fmt.Errorf("coil thread groove: %w", err)
	}
	thread, err := decad.Cut(ctx, cylinder, tool)
	if err != nil {
		return nil, fmt.Errorf("cut external thread: %w", err)
	}
	report, err := doc.Verify(ctx)
	if err != nil {
		return nil, fmt.Errorf("verify external thread: %w", err)
	}
	if len(report.Bodies) != 1 || report.Bodies[0].Validity.Outcome != decad.ValidityValid || report.Status != decad.Suspect {
		return nil, fmt.Errorf("external thread has unexpected verification result: %s", report.Status)
	}
	if len(report.Diagnostics) != 3 {
		return nil, fmt.Errorf("external thread has %d diagnostics, want area, volume and centroid", len(report.Diagnostics))
	}
	area, volume, centroid := false, false, false
	for _, diagnostic := range report.Diagnostics {
		if diagnostic.Code != decad.DiagMeasurementBeyondTolerance {
			return nil, fmt.Errorf("external thread has unexpected diagnostic: %s", diagnostic.Code)
		}
		switch diagnostic.Reading {
		case decad.ReadingArea:
			area = true
		case decad.ReadingVolume:
			volume = true
		case decad.ReadingCentroid:
			centroid = true
		default:
			return nil, fmt.Errorf("external thread has unexpected reading diagnostic: %s", diagnostic.Reading)
		}
	}
	if !area || !volume || !centroid {
		return nil, fmt.Errorf("external thread lacks area, volume or centroid tolerance diagnostics")
	}
	if err := checkThreadGroove(ctx, thread); err != nil {
		return nil, err
	}
	return thread, nil
}

// checkThreadGroove checks the rendered mesh at radius 4.55, safely between
// the 4.1 root and 5 mm crest. Quarter and half turns advance the groove; a stack
// of annular grooves cannot satisfy both the groove and crest checks.
// These are held-mesh parity checks, not certified membership in the true
// solid. They do not change or bypass Body.LocatePoint's certification bounds.
// Verify above independently checks the body's validity and measurement bounds.
func checkThreadGroove(ctx context.Context, body *decad.Body) error {
	mesh, err := body.Tessellate(ctx, units.Millimeters(featureChordTolerance), decad.WithVerification(decad.VerifyNone))
	if err != nil {
		return fmt.Errorf("mesh thread for visual checks: %w", err)
	}
	vertices, triangles := mesh.Vertices(), mesh.Triangles()
	for _, sample := range []struct {
		name       string
		point      r3.Vec
		wantInside bool
	}{
		{"groove", r3.NewVec(4.55, 2.625, 0), false},
		{"next turn", r3.NewVec(4.55, 4.125, 0), false},
		{"quarter-turn groove", r3.NewVec(0, 3, -4.55), false},
		{"quarter-turn crest", r3.NewVec(0, 3.75, -4.55), true},
		{"crest", r3.NewVec(4.55, 3.375, 0), true},
		{"opposite groove", r3.NewVec(-4.55, 3.375, 0), false},
		{"opposite crest", r3.NewVec(-4.55, 2.625, 0), true},
		// This point is on the blank's end plane, inside the cutter's V.
		{"end intersection", r3.NewVec(0, 0, -4.55), false},
		{"end opening", r3.NewVec(0, 0.05, -4.55), false},
	} {
		if sample.name == "end intersection" {
			// Parity alone does not distinguish a cap point from a true opening.
			distanceSquared, err := pointquery.MeshDistanceSquared(ctx, sample.point, vertices, triangles)
			if err != nil {
				return fmt.Errorf("thread mesh end distance: %w", err)
			}
			if distanceSquared.Sign() <= 0 {
				return fmt.Errorf("thread mesh end intersection lies on a facet")
			}
		}
		inside, resolved, err := pointquery.ObliqueParity(ctx, sample.point, vertices, triangles)
		if err != nil {
			return fmt.Errorf("check thread %s: %w", sample.name, err)
		}
		if !resolved || inside != sample.wantInside {
			return fmt.Errorf("thread mesh %s: resolved=%t, inside=%t, want inside=%t",
				sample.name, resolved, inside, sample.wantInside)
		}
	}
	return nil
}

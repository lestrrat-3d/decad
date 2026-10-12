package main

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/solidlens"
	"github.com/lestrrat-3d/units"
)

// threadRender builds the complete eight-turn external thread for the gallery.
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
			models, err := oneModel(ctx, body, cyan, chord)
			if err != nil {
				return solidlens.Scene{}, err
			}
			scene := featureScene(models...)
			scene.Camera = solidlens.Camera{
				Position: solidlens.Vec{X: 30, Y: -45, Z: 25},
				Target:   solidlens.Vec{Y: 10},
				Up:       solidlens.Vec{Z: 1},
				FOV:      30,
			}
			return scene, nil
		},
	}
}

func externalThread(ctx context.Context) (*decad.Body, error) {
	w := sketch.NewWorld()
	axis := decad.SketchLine{Start: decad.Point2{}, End: decad.Point2{V: 1}}
	section, profile, err := sketchLoops(ctx, w, w.XY(), rectangle(0, 0, 5, 20))
	if err != nil {
		return nil, fmt.Errorf("draw thread cylinder: %w", err)
	}
	doc := decad.New()
	cylinder, err := decad.Revolve(ctx, doc, section, profile, axis, decad.FullRevolution{})
	if err != nil {
		return nil, fmt.Errorf("revolve thread cylinder: %w", err)
	}
	const halfWidth = 0.6928203230275509
	groove, grooveProfile, err := sketchLoops(ctx, w, w.XY(), []point{
		{4.1, 3}, {5.3, 3 - halfWidth}, {5.3, 3 + halfWidth},
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
	area, centroid := false, false
	for _, diagnostic := range report.Diagnostics {
		if diagnostic.Code != decad.DiagMeasurementBeyondTolerance {
			return nil, fmt.Errorf("external thread has unexpected diagnostic: %s", diagnostic.Code)
		}
		switch diagnostic.Reading {
		case decad.ReadingArea:
			area = true
		case decad.ReadingCentroid:
			centroid = true
		default:
			return nil, fmt.Errorf("external thread has unexpected reading diagnostic: %s", diagnostic.Reading)
		}
	}
	if !area || !centroid {
		return nil, fmt.Errorf("external thread lacks area or centroid tolerance diagnostics")
	}
	return thread, nil
}

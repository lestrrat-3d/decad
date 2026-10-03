package main

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/r3"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

// The flange is booleanShot's plate: a 96×68×16 mm block with a bore at its
// centre and two bolt holes on its X axis. Its bottom face lies on Z = 0.
const (
	flangeHalfX     = 48.0
	flangeHalfY     = 34.0
	flangeThickness = 16.0
	boreRadius      = 18.0
	boltOffset      = 36.0
	boltRadius      = 7.0
)

// drill is one hole of the flange. Its name names the hole in a
// flangeShape's depths, and the clip's tracks hole.<name>,
// tool.<name>.plunge and tool.<name>.fade.
type drill struct {
	name   string
	x      float64 // the hole's centre on the plate's X axis
	radius float64
}

// drills are the flange's three holes in the order they are cut.
var drills = []drill{
	{name: "bore", x: 0, radius: boreRadius},
	{name: "left", x: -boltOffset, radius: boltRadius},
	{name: "right", x: boltOffset, radius: boltRadius},
}

// A feature whose parameter is below decad's smallest accepted value is left
// out: decad refuses a smaller one.
const (
	minHoleDepth = 0.5
	minFillet    = 0.05
	minChamfer   = 0.5
)

// A through-hole tool clears both plate faces by holeClearance.
const holeClearance = 16.0

// flangeShape is the flange at one stage of its build. A hole depth, a fillet
// radius or a chamfer setback below decad's minimum leaves that feature out.
type flangeShape struct {
	height  float64            // the extrude height
	depths  map[string]float64 // each hole's depth below the top face, by drill name
	fillet  float64            // the radius of the vertical edges' fillet
	chamfer float64            // the setback of the top cap loop's chamfer
}

// flangeBody builds the flange in the order decad accepts: extrude, cut,
// fillet, then the cap-loop chamfer, which no boolean may follow. It cuts the
// through holes first and the blind holes after them: a Cut that follows a
// blind cutter is refused ("requested tolerance ... is below the faceted
// body's minimum mesh bound").
func flangeBody(ctx context.Context, shape flangeShape) (*decad.Body, error) {
	w := sketch.NewWorld()
	doc := decad.New()
	plate, err := prism(ctx, doc, w, w.XY(), shape.height, rectangle(-flangeHalfX, -flangeHalfY, flangeHalfX, flangeHalfY))
	if err != nil {
		return nil, fmt.Errorf("extrude the blank: %w", err)
	}
	blind, through := splitHoles(shape)
	for _, d := range append(through, blind...) {
		tool, err := holeTool(ctx, doc, w, d, shape.height, shape.depths[d.name])
		if err != nil {
			return nil, fmt.Errorf("hole %s: %w", d.name, err)
		}
		plate, err = decad.Cut(ctx, plate, tool)
		if err != nil {
			return nil, fmt.Errorf("drill hole %s: %w", d.name, err)
		}
	}
	if shape.fillet >= minFillet {
		plate, err = plate.Fillet(ctx, decad.Edges(decad.ParallelTo(r3.NewVec(0, 0, 1))), units.Millimeters(shape.fillet))
		if err != nil {
			return nil, fmt.Errorf("fillet the plate: %w", err)
		}
	}
	if shape.chamfer >= minChamfer {
		plate, err = plate.Chamfer(ctx, decad.Edges(decad.CreatedBy(decad.CapEnd(plate))), units.Millimeters(shape.chamfer))
		if err != nil {
			return nil, fmt.Errorf("chamfer the cap loop: %w", err)
		}
	}
	return plate, nil
}

// throughFlange is the flange at full height with every hole drilled
// through, and no fillet or chamfer.
func throughFlange() flangeShape {
	depths := make(map[string]float64, len(drills))
	for _, d := range drills {
		depths[d.name] = flangeThickness + holeClearance
	}
	return flangeShape{height: flangeThickness, depths: depths}
}

// splitHoles sorts the shape's holes, in drill order, into blind holes and
// through holes, and returns them in that order, leaving out the holes too
// shallow to cut.
func splitHoles(shape flangeShape) ([]drill, []drill) {
	var blind, through []drill
	for _, d := range drills {
		depth := shape.depths[d.name]
		switch {
		case depth < minHoleDepth:
		case depth > shape.height-minHoleDepth:
			through = append(through, d)
		default:
			blind = append(blind, d)
		}
	}
	return blind, through
}

// holeTool is the cutter for a hole depth below the top face of a plate top
// millimetres tall, depth at least minHoleDepth: a blind cutter whose end cap
// lies at that depth up to top - minHoleDepth, and a through cutter clearing
// both faces above it. No cutter ends in a plate face: decad refuses a cap
// lying in the top face, and one lying in the bottom face gives the wrong
// body.
func holeTool(ctx context.Context, doc *decad.Document, w *sketch.World, d drill, top, depth float64) (*decad.Body, error) {
	if depth > top-minHoleDepth {
		// A through cutter is sketched on XY itself: with three cutters
		// sketched on an offset plane, decad refuses the third Cut ("requested
		// tolerance ... is below the faceted body's minimum mesh bound").
		return cylinder(ctx, doc, w, w.XY(), point{d.x, 0}, d.radius, decad.Symmetric{D: units.Millimeters(top + holeClearance)})
	}
	bottom := top - depth
	plane, err := w.CreateOffsetPlane(w.XY(), bottom)
	if err != nil {
		return nil, err
	}
	return cylinder(ctx, doc, w, plane, point{d.x, 0}, d.radius, decad.Distance{
		D:   units.Millimeters(top + holeClearance - bottom),
		Dir: decad.Along,
	})
}

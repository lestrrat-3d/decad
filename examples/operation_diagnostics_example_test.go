package examples_test

import (
	"context"
	"errors"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/sketch"
	"github.com/lestrrat-3d/units"
)

func Example_operation_diagnostics() {
	ctx := context.Background()
	w := sketch.NewWorld()
	s, err := w.CreateSketch(w.XY())
	if err != nil {
		fmt.Printf("failed to create sketch: %s\n", err)
		return
	}
	rect := s.CreateRectangle(0, 0, 10, 10)
	s.Fix(rect.A)
	if _, err := s.Solve(ctx); err != nil {
		fmt.Printf("failed to solve sketch: %s\n", err)
		return
	}
	doc := decad.New()
	body, err := decad.Extrude(ctx, doc, s, s.Profiles()[0],
		decad.Distance{D: units.Millimeters(5), Dir: decad.Along})
	if err != nil {
		fmt.Printf("failed to extrude: %s\n", err)
		return
	}
	fmt.Printf("operation: %s\n", doc.OperationHistory().Operations()[0].Kind())
	fmt.Printf("warnings: %d\n", len(body.OperationDiagnostics()))

	_, err = decad.Edges(decad.Free()).SelectEdges(body)
	if err == nil {
		fmt.Println("selection succeeded")
		return
	}
	err = fmt.Errorf("check body: %w", err)
	var diagnostics decad.OperationDiagnosticError
	if errors.As(err, &diagnostics) {
		for _, diagnostic := range diagnostics.OperationDiagnostics() {
			switch diagnostic.Code {
			case decad.OpDiagSelectionFailed:
				fmt.Printf("failure: %s\n", diagnostic.Code)
			default:
				fmt.Printf("other failure: %s\n", diagnostic.Code)
			}
		}
	}
	// Output:
	// operation: extrude
	// warnings: 0
	// failure: selection_failed
}

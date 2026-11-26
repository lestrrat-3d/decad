package examples_test

import (
	"context"
	"fmt"

	"github.com/lestrrat-3d/decad"
	"github.com/lestrrat-3d/units"
)

// SheetMetal builds a bent strip of sheet metal from its flat legs and the angles between
// them, and reports the blank it is cut from. Here a 30 mm and a 20 mm leg of 2 mm steel,
// 10 mm wide, meet in a right-angle bend of 3 mm inside radius.
func Example_sheet_metal() {
	bracket, report, err := decad.SheetMetal(decad.New(), units.Millimeters(2), units.Millimeters(3), units.Millimeters(10), []units.Value{units.Millimeters(30), units.Millimeters(20)}, []units.Value{units.Degrees(90)}).Do(context.Background())
	if err != nil {
		fmt.Printf("failed to build the strip: %s\n", err)
		return
	}
	volume, err := bracket.Volume()
	if err != nil {
		fmt.Printf("failed to measure volume: %s\n", err)
		return
	}
	fmt.Printf("volume: %.0f mm^3, blank: %.2f mm long\n", volume.Value.Base(), report.FlatLength.Base())
	// Output:
	// volume: 1126 mm^3, blank: 56.09 mm long
}

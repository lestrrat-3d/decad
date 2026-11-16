package prismcells

// disableGuards switches the named arrangement guards off and returns the
// function that restores them.
func disableGuards(names ...string) func() {
	disabledGuards.Store(&names)
	return func() { disabledGuards.Store(nil) }
}

package meshbool

// Only the serial disposable latency test sets this hook. No production caller does.
var validationConformSplitHook func()

func validationConformBeforeSplit() {
	if validationConformSplitHook != nil {
		validationConformSplitHook()
	}
}

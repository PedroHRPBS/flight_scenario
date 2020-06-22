# flight_scenario
Flight Scenario is a high level command architecture.

It's based on Flight Elements, which are single task elements, connected together to create a Flight Pipeline, which generates high level controls to the aircraft, e.g., taking off, flying in a defined path, going back to launch point and land.

Flight Pipelines run on dedicated threads creating a full Mission Management solution.

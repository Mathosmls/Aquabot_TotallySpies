from aquabot_python.scenario_container import ScenarioContainer, WindturbineState
from aquabot_python.environnement_container import generate_environnement
from aquabot_python.scenario_parser import serialize, deserialize
from aquabot_python.scenario_pyplot import plot_scenario
from aquabot_python.generator_methods import get_random_windturbine, get_distance_between_points, get_random_windturbines_states
import random
import string

# Constants
NUMBERS_OF_WINDTURBINES = 3

def generate_scenario(prefix_name="scenario", seed=None, obstacles=True):
    # Generate a random seed if not sepcified
    if (seed is None):
        letters = string.ascii_lowercase + string.ascii_uppercase + string.digits
        seed = ''.join(random.choice(letters) for i in range(16))
    print("Generation seed : " + str(seed))
    random.seed(seed)
    # Create environnement container
    environnement_container = generate_environnement(obstacles)
    # Create scenario container
    scenario_container = ScenarioContainer([])
    # Generate list of turbine states and turbines
    states = get_random_windturbines_states(NUMBERS_OF_WINDTURBINES)
    for state in states:
        scenario_container.windturbines.append(get_random_windturbine(state, environnement_container, scenario_container))
    # Parse scenario to xml
    serialize(scenario_container, prefix_name + ".sdf", seed=seed)
    # Plot scenario
    plot_scenario(scenario_container, environnement_container, prefix_name + ".png")

def reopen_scenario(scenario_name="scenario.sdf", obstacles=True):
    # Parse scenario from xml
    scenario_container = deserialize(scenario_name)
    # Create environnement container
    environnement_container = generate_environnement(obstacles)
    # Plot scenario
    prefix_name = scenario_name.replace(".sdf","")
    plot_scenario(scenario_container, environnement_container, prefix_name + "_reopened.png")

def regenerate_scenario_file(scenario_name="scenario.sdf", obstacles=True):
    # Parse scenario from xml
    scenario_container = deserialize(scenario_name)

    # Create environnement container
    environnement_container = generate_environnement(obstacles)

    # Change what you want in scenario_container here...
    # example adding a new windturbine if there is less than 4
    if len(scenario_container.windturbines) < 4:
        scenario_container.windturbines.append(get_random_windturbine(WindturbineState.KO, environnement_container, scenario_container))

    prefix_name = scenario_name.replace(".sdf","")

    # Parse scenario to xml
    serialize(scenario_container, prefix_name + "_regenerated.sdf")

    # Plot scenario
    plot_scenario(scenario_container, environnement_container, prefix_name + "_regenerated.png")

if __name__ == "__main__":
    generate_scenario()
import matplotlib.pyplot as plt
from IPython.display import display, clear_output
import time as systime  # to pause for visual updates
import math

# Setup
solar_flux = 590  # W/m^2
day_cycle = 0.5  # Fraction of time in sunlight
solar_area = 0.1  # m^2
panel_efficiency = 0.3
battery_capacity = float(100)  # Wh
power_usage_move = 40  # W
power_usage_idle = 2  # W
initial_battery_charge = float(50)  # Wh
cycle_duration = 1479  # Minutes in a Martian day (Sol)
resolution = 10  # Minutes
speed = 0.14 * 1000 / 60  # m/min

# Initial conditions
time = 0
distance = 0
time_of_sol = 540  # Minutes (initially 9 AM)
sun = "yes"
battery_level = initial_battery_charge

# Calculated parameters
power_generated = solar_flux * solar_area * panel_efficiency # W

# Functions
def charge_rate(battery_level, activity):
    if activity == "move":
        return 99999
    elif activity == "idle":
        return 3 * math.e ** (-3 * (-math.log(1 - (battery_level / battery_capacity)) / 3)) * 60 + 1

def battery_gain(dt, sun, activity):
    if sun == "yes":
        return min(power_generated * dt / 60, charge_rate(battery_level, activity) * dt / 60)
    else:
        return 0

def battery_drain(dt, activity):
    if activity == "move":
        return power_usage_move * dt / 60
    elif activity == "idle":
        return power_usage_idle * dt / 60
    else:
        return power_usage_move * dt / 60

# Data for plotting
time_history = [time]
battery_history = [battery_level]

def mini_loop(activity):
    global time, sun, battery_level, time_of_sol
    if time_of_sol < 360 or time_of_sol >= 1080:
        sun = "no"
    else:
        sun = "yes"

    # Compute changes
    battery_level += battery_gain(resolution, sun, activity) - battery_drain(resolution, activity)
    if battery_level >= battery_capacity:
        battery_level = battery_capacity

    if time_of_sol >= cycle_duration:
        time_of_sol -= cycle_duration

    # Store data for graph
    time_history.append(time + resolution)
    battery_history.append(battery_level)

    # Plot live
    plt.clf()
    plt.title("Battery Level vs Time")
    plt.xlabel("Time (minutes)")
    plt.ylabel("Battery Level (Wh)")
    plt.grid(True)
    plt.plot(time_history, battery_history, marker=" ")
    plt.ylim(0, battery_capacity)
    plt.show(block=False)
    plt.pause(0.1)

    if battery_level <= 0:
        print("\nRover stranded! You FAILED\n")
        exit()
    elif battery_level == battery_capacity:
        print("There's much exploring to do!")

# Main loop
def loop():
    global time, sun, battery_level, time_of_sol
    plt.ion()  # Interactive mode on

    while True:
        clear_output(wait=True)

        display_hour = int(time_of_sol // 60)
        display_minute = int(time_of_sol % 60)

        # Display status
        print(f"\nCurrent Martian Time: {display_hour:02d}:{display_minute:02d}")
        print(f"Battery level: {battery_level:.2f} Wh")

        # Ask for user input
        activity = input("Enter activity (move/idle) or 'exit' to quit: ").strip().lower()
        if activity == "exit":
            break
        try:
            time_increment = float(input(f"Enter time increment (multiple of {resolution}) minutes: "))
        except ValueError:
            print("Invalid input, please enter a number.")
            continue
        if activity == "yes":
            distance += speed * time_increment

        for i in range(int(time_increment/resolution)):
            mini_loop(activity)
            time += resolution
            time_of_sol += resolution

    plt.ioff()
    plt.show()

if __name__ == "__main__":
    print("\nYou are in control of the Mars rover (you will have sunlight from 6:00 to 18:00 of the 24 hour 39 minute Martian day).")
    loop()

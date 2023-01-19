import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

def main() -> int:
    dfs = [] # Hols data frames

    # Read in data into dataframe
    df_play = pd.read_csv("play/01-27-2021_07-26-19_bd5ac749_Bernie.csv")
    df_sleep = pd.read_csv("sleep/02-28-2021_22-03-28_7aa60c8f_Molly.csv")
    df_seizure = pd.read_csv("seizure/02-02-2021_14-50-33_bd5ac749_Mabel.csv")
    dfs = [df_play, df_sleep, df_seizure]

    # Values for the plots
    attributes = ['g_x', 'g_y', 'g_z', 'a_x', 'a_y', 'a_z'] # List of sensor data to plot
    titles = ['play', 'sleep', 'seizure']
    x_label = 'time'

    # Plot each dataframe
    for title_i, df in enumerate(dfs):
        fig, (ax1, ax2, ax3, ax4, ax5, ax6) = plt.subplots(6, 1) # Create the subplots
        subplots = [ax1, ax2, ax3, ax4, ax5, ax6] # List of subplots

        # Clean data
        df = df.dropna().drop(columns=["time"])
        df['time'] = range(1, len(df) + 1) # Each index is 1/16 of a second
        df['time'] /= 16

        # Plot all the data
        for i in range(len(attributes)):
            sns.lineplot(x=x_label, y=attributes[i], data=df, ax=subplots[i])
        
        # Remove all x-axis except the bottom one
        for ax in [plot for plot in subplots if plot != ax6]:
            ax.set_xlabel("")
            ax.set_xticklabels([])
            ax.set_xticks([])

        ax1.set_title(titles[title_i]) # Set title for plot

        # Show the plot
        plt.show()

    return 0

main()
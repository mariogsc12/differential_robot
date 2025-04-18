from pathlib import Path
import yaml
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator,MultipleLocator

Log_Message = {'ERROR'  : "[ ERROR   ]",
           'WARNING': "[ WARNING ]",
           'INFO'   : "[ INFO    ]"}

def get_results_path():
    """ Builds the path to the Results folders. """

    file_path = Path(__file__).parent.parent
    results_path = file_path / 'Results' 
    return results_path.resolve()

def get_test_path(test_name):
    """ Builds the path to the test folder using the given test name. """

    result_path = get_results_path()
    test_path = result_path / test_name 
    return test_path.resolve()

def get_data_path(test_name):
    """ Builds the path to the test/data folder using the given test name. """

    test_path = get_test_path(test_name)
    data_path = test_path / "Data"
    return data_path.resolve()

def get_plots_path(test_name):
    """ Builds the path to the test/plots folder using the given test name. """

    test_path = get_test_path(test_name)
    plots_path = test_path / "Plots"
    return plots_path.resolve()

def create_output_folders(test_name):
    """Create the output folders if they don't already exist."""
    results_path = get_results_path()
    test_path = results_path / test_name
    plots_path = test_path / "Plots"
    data_path = test_path / "Data"

    results_path.mkdir(parents=True, exist_ok=True)
    test_path.mkdir(parents=True, exist_ok=True)
    plots_path.mkdir(parents=True, exist_ok=True)
    data_path.mkdir(parents=True, exist_ok=True)

    return data_path

def message_to_string(names, divided_message):
    out_msg = ""
    
    for i in range(0,len(names)):
        out_msg += names[i] + ": " + str(divided_message[i])
        out_msg += " - "

    return out_msg


class ConfigReader:

    def __init__(self):
        self.path = Path(__file__)
        self.path_parent = self.path.parent
        self.path_yaml = self.path_parent / 'config.yaml' 
    
    def process(self):
        """ Main function of the class to read and store yaml file """
        if not self.path_yaml.exists():
            print(f"{Log_Message['ERROR']}: The file '{self.path_yaml}' was not found.")
            return None

        with open(self.path_yaml, 'r') as file:
            self.data = yaml.safe_load(file)

        self.check_config()

    def check_config(self):
        if len(self.data['main']['data']) <= 1:
            print(f"{Log_Message['ERROR']}: There are not enough data configured")
        elif self.data['main']['data'][0].lower() != "time":
            self.data['main']['data'][0] = "time"
            print(f"{Log_Message['WARNING']}: The first input should be the time")

        if len(self.data['main']['data']) != len(self.data['main']['data_labels']):
            print(f"{Log_Message['WARNING']}: The input data is not correctly configured.")
            self.data_length = 0
        else:
            self.data_length = len(self.data['main']['data']) - 1   # Not considering time 


class Plot:
    def __init__(self):
        # Load config
        reader = ConfigReader()
        reader.process()
        self.config = reader.data
        self.data_length = reader.data_length

        self.set_horizontal_style()


    def set_horizontal_style(self):
        """ Define the plot size in inches """
        if self.config['plot']['mode_horizontal'] == True:
            self.input = 'horizontal'
        else:
            self.input = 'default'

        self.width = self.config['plot'][self.input]['width']
        self.height = self.config['plot'][self.input]['height']
        self.update_default_style()

    def update_default_style(self):
        plt.rcParams.update({
            "font.family": self.config['plot']['text_font'],  
            "font.size": self.config['plot']['text_size'],
            "axes.labelsize": self.config['plot']['text_size'],
            "xtick.labelsize": self.config['plot']['text_size'],
            "ytick.labelsize": self.config['plot']['text_size'],
        })
    
    def set_plot_style(self, ax, df):
            
        ax.xaxis.set_major_locator(MaxNLocator(integer=True))

        plt.xlim(left=df["time"].iloc[0], right=df["time"].iloc[-1])
        plt.xlabel(self.config['plot']['x_label'])
        
        if self.config['plot'].get('y_limits') is not None:
            ax.set_ylim(bottom=self.config['plot']['y_limits'][0], top=self.config['plot']['y_limits'][1])

        if self.config['plot'].get('y_ticks') is not None:
            ax.set_yticks(self.config['plot']['y_ticks'])

        if self.config['plot'].get('title') is not None:
            plt.title(self.config['plot']['title'])

        plt.grid(True)
        

    def set_legend(self):
        if self.data_length > 2:
            legend_location = 'upper left'
            legend_bbox_to_anchor = (1.0,0.5)
        else:
            legend_location = self.config['plot']['legend_location']
            legend_bbox_to_anchor = None

        legend = plt.legend(
            loc=legend_location,                                    # Locate the legend 
            bbox_to_anchor=legend_bbox_to_anchor,                   # Relocate the legend 
            # ncol=legend_ncol,                                       # Number of columns
            frameon=True,                                           # Enables the border
            framealpha=1,                                           # Transparency (should be 1 for eps format)
            fontsize=self.config['plot']['text_size'] -2,           # Font size
            borderpad=0.5,                                          # Space between the border and the text
            handlelength=1.5,                                       # Length of the handles (lines) in the legend
            labelspacing=0.5,                                       # Spacing between the labels
            borderaxespad=1,                                        # Space between the border of the legend and the plot
            facecolor='white',                                      # Background color of the legend
            edgecolor='black',                                      # Border color
            title=None,                                             # Title of the legend (optional)
            title_fontsize=None,                                    # Font size for the title
            shadow=False                                            # Shadow for the legend
        )

        border_width = plt.gca().spines['bottom'].get_linewidth()
        legend.get_frame().set_linewidth(border_width)
        






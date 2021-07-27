# WhudStateMachine教程

为了简单地使用WhudStateMachine，我们只需要配置**config**文件夹中的文件。

<img src="images\image-20210612121901791.png" alt="image-20210612121901791" style="zoom: 67%;" />

我们可以在**config**文件夹中看到三个文件。main_task.yaml包含顺序执行的主要任务，interrupt_task.yaml包含每个主任务的中断任务。plugin_params.yaml包含插件的参数。我们首先看看如何设置主要任务。

## 设定主任务

首先打开main_task.yaml。

<img src="images\image-20210612123032421.png" alt="image-20210612123032421" style="zoom: 80%;" />

现在，让我们来分析一下代码。

<img src="images\image-20210612123945198.png" alt="image-20210612123945198" style="zoom:80%;" />

如果我们想要添加一些主任务(例如:task_A和task_B)，我们可以按照第10行和第11行的格式写出主任务名称。注意，task_A和task_B只是用户自定义的名称，任务对应的插件和参数将在下面定义。

<img src="images\image-20210612124039207.png" alt="image-20210612124039207" style="zoom:80%;" />

这是我们主任务的描述。第16行是我们将要配置的任务的名称(**必须与main_task_list**中定义的名称相同)。第17行指定任务的插件类型。第18行是此任务的最大执行时间(**单位为秒，且必须为整数**)。当时间结束时，它将强制跳转到下一个主任务。第19行到第21行指定插件所需的参数(**对于不同的插件参数的类型和数量是不同的**)。第22行是任务名称，必须与main_task_list和主任务描述中定义的名称相同(第10行和第16行)。第23行是中断任务的名称。和主任务的名称一样，它也是一个用户定义的名称。中断任务在interrupt_task.yaml中配置。

## 设置中断任务

打开interrupt_task.yaml

<img src="images\image-20210614234549948.png" alt="image-20210614234549948" />

它几乎与main_task.yaml一模一样，仅在第10行中有一个区别:

<img src="images\image-20210614234659188.png" alt="image-20210614234659188" />

此处我们让中断任务完成时，它可以决定跳转到哪个后续主任务，**return_name**是当中断任务完成时将跳转到的主任务的名称。此外，中断任务也有可能在规定计时期间内并未完成。在这种情况下，它将自动跳转回触发该中断任务的主任务，并且在该主任务中，中断任务被禁用(它将不会被再次触发)。

# An simple tutorial for WhudStateMachine

In order to simply use WhudStateMachine, we just need to configure the files in folder **config**.

<img src="images\image-20210612121901791.png" alt="image-20210612121901791" style="zoom: 67%;" />

We can see three files in **config** folder. As main_task.yaml contains main tasks that are executed sequentially, interrupt_task.yaml contains interrupt tasks for each main task. plugin_params.yaml contains parameters for plugins. We first take a look at how to set main tasks.

## Set main tasks

Let's open main_task.yaml first.

<img src="images\image-20210612123032421.png" alt="image-20210612123032421" style="zoom: 80%;" />

Now, let's break the code down.

<img src="images\image-20210612123945198.png" alt="image-20210612123945198" style="zoom:80%;" />

If we want to add some main tasks(i.e. task_A and task_B), we can write the names of tasks in format as line 10 and 11. Note that task_A and task_B are only the names specified by users, their corresponding plugins and parameters will be defined following.

<img src="images\image-20210612124039207.png" alt="image-20210612124039207" style="zoom:80%;" />

Here is the description for our main tasks. Line 16 is the name of task which we are going to configure(**must be same as names defined in main_task_list**). Line 17 specifies the  plugin type of task. Line 18 is the maximum executing time for this task(**in seconds and must be integer**). When time is finished it will jump to the next task. Line 19 to line 21 specify the parameters needed for plugins(**which are different for different plugins**). Line 22 is the task name which must be same as name defined in main_task_list and main task description(line 10 and line 16). Line 23 is the name of interrupt task. Just like the name of main task, it's also a user-defined name. Its detailed description is in interrupt_task.yaml. 

## Set interrupt task

Let's open the interrupt_task.yaml

<img src="images\image-20210614234549948.png" alt="image-20210614234549948" />

It's almost the same as the main_task.yaml, with only one difference in line 10:

<img src="images\image-20210614234659188.png" alt="image-20210614234659188" />

 We assume that when interrupt task is done, it can decide which main task it will jump to, and the **return_name** is the name of task which will be jumped to  when interrupt task is done.  It also has this probability that interrupt task is not done when time is finished. In this case it will jump to the main task in which the interrupt task is stimulated, and interrupt task is disabled(it will not be stimulated again) until next main task.
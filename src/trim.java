import java.io.*;
import java.lang.*;
import java.util.ArrayList;

public class trim
{
	public static void main(String[] args)
	{
		try
		{
			Integer percent_remove_both_ends = Integer.parseInt(args[0]);
			ArrayList<String> data_list = new ArrayList<String>();
			BufferedReader reader = new BufferedReader(new FileReader("train.grt.trim"));
			BufferedWriter writer = new BufferedWriter(new FileWriter("train.grt.trim.result"));
            writer.write("GRT_LABELLED_TIME_SERIES_CLASSIFICATION_DATA_FILE_V1.0\n");
            writer.write("DatasetName: NOT_SET\n");
            writer.write("InfoText:\n");
            writer.write("NumDimensions: 4\n");
            writer.write("TotalNumTrainingExamples: 2286\n");
            writer.write("NumberOfClasses: 9\n");
            writer.write("ClassIDsAndCounters:\n");
            writer.write("1   300\n");
            writer.write("2   300\n");
            writer.write("3   301\n");
            writer.write("4   299\n");
            writer.write("5   299\n");
            writer.write("6   199\n");
            writer.write("7   199\n");
            writer.write("8   196\n");
            writer.write("9   193\n");
            writer.write("UseExternalRanges: 0\n");
            writer.write("LabelledTimeSeriesTrainingData:\n");
			String line;
			int tms = 0;
			while ((line = reader.readLine())!= null)
			{
				if (line.equals("************TIME_SERIES************"))
				{
					tms++;
					data_list.clear();
					String line_classID = reader.readLine();
					String line_length = reader.readLine();
					String[] line_length_split = line_length.split(": ");
					String line_data = reader.readLine();
					Integer length = Integer.parseInt(line_length_split[1]);
					Integer cut = (int)((double)length * ((double)(percent_remove_both_ends)) / 100.0);
					if (tms%100 == 0)
					{
						System.out.print("cut = " + cut.toString() + " ");
						System.out.println(tms);
					}
					for (int i=0; i<cut; i++)
						reader.readLine();
					for (int i=cut; i<length-cut; i++)
						data_list.add(reader.readLine());
					for (int i=length-cut; i<length; i++)
						reader.readLine();
					writer.write("************TIME_SERIES************\n");
					writer.write(line_classID + "\n");
					writer.write("TimeSeriesLength: ");
					Integer l = length-cut*2;
					writer.write(l.toString());
					writer.write("\nTimeSeriesData:\n");
					int new_len = data_list.size();
					for (int i=0; i<new_len; i++)
						writer.write(data_list.get(i)+"\n");
				}
			}
			
			reader.close();
			writer.close();
			
		}catch (Exception e){e.printStackTrace();}
	}
}

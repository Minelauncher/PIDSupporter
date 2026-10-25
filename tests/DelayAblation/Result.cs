namespace PIDSupporter {
    internal sealed class TuneResult
    {
        public bool accepted { get; set; }
        public string reason { get; set; } = "";
        public double[]? pid { get; set; }
        public double validation { get; set; }
        public double predicted_ratio { get; set; }
        public double seconds { get; set; }
        public string message { get; set; } = "";
        public int accumulated_records { get; set; }
        public bool needs_more_data { get; set; }
    }


}
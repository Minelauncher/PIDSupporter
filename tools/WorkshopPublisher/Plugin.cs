using System;
using System.IO;
using System.Linq;
using System.Security.Cryptography;
using BrilliantSkies.Core.Timing;
using BrilliantSkies.Modding;
using Newtonsoft.Json.Linq;
using Steamworks;
using UnityEngine;

// Development-only uploader. This assembly is never part of the public mod.
public sealed class WorkshopPublisher : GamePlugin
{
    public string name => "PIDSupporter Workshop publisher";
    public System.Version version => new System.Version(0, 1);
    static string root, jobPath, statePath, content, description, title;
    static JObject job, state;
    static PublishedFileId_t item;
    static UGCUpdateHandle_t update;
    static UGCQueryHandle_t query;
    static CallResult<CreateItemResult_t> createCall;
    static CallResult<SubmitItemUpdateResult_t> uploadCall;
    static CallResult<SteamUGCQueryCompleted_t> queryCall;
    static Callback<DownloadItemResult_t> downloadCall;
    static bool begun, done, verifying, uploading;
    static float started, lastProgress;
    static uint existingPreviews, originalPreviews;
    static readonly string[] allowed = { "plugin.json", "header.header", "AUTOTUNE.txt", "AUTOTUNE_THEORY_KO.md", "AUTOTUNE_THEORY_EN.md", "PIDSupporterCore.dll", "PIDSupporterSelector.dll", "0Harmony.dll", "header.jpg" };
    public void OnSave() { }
    public void OnLoad()
    {
        root = Path.GetFullPath(Path.Combine(Path.GetDirectoryName(typeof(WorkshopPublisher).Assembly.Location), "../../../.."));
        jobPath = File.ReadAllText(Path.Combine(root, "publisher-job-path.txt")).Trim();
        job = JObject.Parse(File.ReadAllText(jobPath));
        statePath = (string)job["statePath"];
        state = File.Exists(statePath) ? JObject.Parse(File.ReadAllText(statePath)) : new JObject();
        ulong.TryParse((string)state["itemId"], out var savedId); item = new PublishedFileId_t(savedId);
        Application.runInBackground = true;
        GameEvents.UpdateEvent.RegWithEvent(Tick);
    }
    static void Require(bool ok, string reason) { if (!ok) throw new InvalidOperationException(reason); }
    static string Hash(string path) { using (var sha = SHA256.Create()) using (var file = File.OpenRead(path)) return BitConverter.ToString(sha.ComputeHash(file)).Replace("-", ""); }
    static void Save(string step)
    {
        state["step"] = step; state["updatedAt"] = DateTimeOffset.UtcNow.ToString("O");
        state["itemId"] = item.m_PublishedFileId.ToString(); state["title"] = title;
        state["url"] = item == PublishedFileId_t.Invalid ? "" : "https://steamcommunity.com/sharedfiles/filedetails/?id=" + item;
        File.WriteAllText(statePath, state.ToString());
        File.WriteAllText(Path.Combine(root, "publisher-progress.json"), state.ToString());
    }
    static void Finish(string step, string detail = null)
    {
        state["detail"] = detail ?? ""; Save(step); done = true;
        File.WriteAllText(Path.Combine(root, "publisher-done.json"), state.ToString());
        Application.Quit();
    }
    static void Fail(Exception e) => Finish("failed", e.GetBaseException().Message);
    static void ValidateFiles()
    {
        content = Path.GetFullPath((string)job["content"]); title = (string)job["title"];
        description = File.ReadAllText((string)job["description"]);
        Require(title == "PIDSupporter", "Unexpected upload title");
        Require(Directory.GetDirectories(content).Length == 0, "Content may not contain subdirectories");
        Require(Directory.GetFiles(content).Select(Path.GetFileName).OrderBy(x => x).SequenceEqual(allowed.OrderBy(x => x)), "Content must contain exactly the nine approved release files");
        foreach (var pair in (JObject)job["hashes"]) Require(Hash(Path.Combine(content, pair.Key)) == (string)pair.Value, "Release hash mismatch: " + pair.Key);
        var metadata = JObject.Parse(File.ReadAllText(Path.Combine(content, "plugin.json")));
        Require((string)metadata["name"] == "PIDSupporter" && (string)metadata["version"] == (string)job["version"], "Unexpected release metadata");
        Require(description.Split('\n').Any(s => s.Trim() == "Mod latest version " + (string)job["version"]), "Missing matching update marker");
        Require(System.Text.Encoding.UTF8.GetByteCount(description) < 8000, "Description exceeds the game's Steam details buffer");
        foreach (string path in ((JArray)job["gallery"]).Values<string>().Concat(new[] { (string)job["preview"] }))
            Require(File.Exists(path) && new FileInfo(path).Length < 1000000, "Missing/oversized preview image");
    }
    static void Tick(ITimeStep step)
    {
        if (done) return;
        try
        {
            if (!begun)
            {
                if (Time.realtimeSinceStartup < 12) return;
                begun = true; started = Time.realtimeSinceStartup;
                ValidateFiles();
                Require(SteamWrapper.Initialized && SteamUser.BLoggedOn(), "Steam is not connected. Sign in to the Steam desktop client and retry.");
                Require(SteamUtils.GetAppID().m_AppId == 268650, "Publisher must run inside From the Depths (268650)");
                state["ownerSteamId"] = SteamUser.GetSteamID().m_SteamID.ToString();
                ulong.TryParse((string)state["itemId"], out var id); item = new PublishedFileId_t(id);
                Require(id == 3735379602UL, "Only the existing PIDSupporter item may be updated");
                Query(false); return;
            }
            if (Time.realtimeSinceStartup - started > 240)
            {
                Finish(item == PublishedFileId_t.Invalid ? "create-outcome-unknown" : "outcome-unknown", "Timed out; use the saved item ID when resuming. Do not create a duplicate."); return;
            }
            if (uploading && Time.realtimeSinceStartup - lastProgress > 5)
            {
                lastProgress = Time.realtimeSinceStartup;
                var progress = SteamUGC.GetItemUpdateProgress(update, out var processed, out var total);
                state["uploadStatus"] = progress.ToString(); state["bytesProcessed"] = processed; state["bytesTotal"] = total; Save("uploading");
            }
        }
        catch (Exception e) { Fail(e); }
    }
    static void Created(CreateItemResult_t result, bool io)
    {
        try
        {
            if (result.m_nPublishedFileId != PublishedFileId_t.Invalid) item = result.m_nPublishedFileId;
            if (io) { Finish("create-outcome-unknown", "Steam CreateItem I/O failure. Resolve the created item before retrying."); return; }
            Require(result.m_eResult == EResult.k_EResultOK && item != PublishedFileId_t.Invalid, "CreateItem: " + result.m_eResult);
            Save("created");
            if (result.m_bUserNeedsToAcceptWorkshopLegalAgreement) { Finish("agreement-required", "Accept the Steam Workshop legal agreement, then resume this item."); return; }
            Query(false);
        }
        catch (Exception e) { Fail(e); }
    }
    static void Query(bool verify)
    {
        verifying = verify;
        query = SteamUGC.CreateQueryUGCDetailsRequest(new[] { item }, 1);
        Require(query != UGCQueryHandle_t.Invalid, "Could not create item query");
        Require(SteamUGC.SetAllowCachedResponse(query, 0) && SteamUGC.SetReturnLongDescription(query, true)
            && SteamUGC.SetReturnAdditionalPreviews(query, true), "Could not configure item query");
        queryCall = new CallResult<SteamUGCQueryCompleted_t>(Queried);
        var request = SteamUGC.SendQueryUGCRequest(query);
        Require(request != SteamAPICall_t.Invalid, "Steam rejected item query"); queryCall.Set(request);
        Save(verify ? "verifying" : "checking-owner");
    }
    static void Queried(SteamUGCQueryCompleted_t result, bool io)
    {
        try
        {
            Require(!io && result.m_eResult == EResult.k_EResultOK && result.m_unNumResultsReturned == 1, "Item query failed: " + result.m_eResult);
            Require(SteamUGC.GetQueryUGCResult(query, 0, out var details) && details.m_eResult == EResult.k_EResultOK, "Item details unavailable");
            Require(details.m_nPublishedFileId == item && details.m_nConsumerAppID.m_AppId == 268650
                && details.m_ulSteamIDOwner == SteamUser.GetSteamID().m_SteamID, "Item does not belong to this account and game");
            existingPreviews = SteamUGC.GetQueryUGCNumAdditionalPreviews(query, 0);
            SteamUGC.ReleaseQueryUGCRequest(query); query = UGCQueryHandle_t.Invalid;
            if (!verifying) { originalPreviews = existingPreviews; Upload(); return; }
            Require(details.m_rgchTitle == title && details.m_rgchDescription.Trim() == description.Trim(), "Published title or description mismatch");
            Require(details.m_eVisibility == ERemoteStoragePublishedFileVisibility.k_ERemoteStoragePublishedFileVisibilityPublic && !details.m_bBanned, "Item is not publicly visible");
            Require(existingPreviews == originalPreviews + ((JArray)job["gallery"]).Count, "Published gallery count mismatch");
            state["verifiedTitle"] = details.m_rgchTitle; state["galleryCount"] = existingPreviews; state["visibility"] = "public";
            state["publishedBytes"] = details.m_nFileSize;
            Save("downloading-verification");
            downloadCall = Callback<DownloadItemResult_t>.Create(Downloaded);
            Require(SteamUGC.DownloadItem(item, true), "Published, but verification download could not start");
        }
        catch (Exception e) { if (query != UGCQueryHandle_t.Invalid) SteamUGC.ReleaseQueryUGCRequest(query); Fail(e); }
    }
    static void Upload()
    {
        // Only the public staging metadata is modified here, never a user's installed mod directory.
        string metadataPath = Path.Combine(content, "plugin.json");
        var metadata = JObject.Parse(File.ReadAllText(metadataPath)); metadata["workshop_id"] = item.m_PublishedFileId.ToString();
        File.WriteAllText(metadataPath, metadata.ToString(Newtonsoft.Json.Formatting.None) + "\n");
        update = SteamUGC.StartItemUpdate(new AppId_t(268650), item);
        Require(update != UGCUpdateHandle_t.Invalid, "Invalid update handle");
        Require(SteamUGC.SetItemUpdateLanguage(update, "english"), "Could not set language");
        Require(SteamUGC.SetItemTitle(update, title) && SteamUGC.SetItemDescription(update, description), "Could not set title/description");
        Require(SteamUGC.SetItemTags(update, new[] { "#mod", "Plugin", "Utility" }), "Could not set FTD mod tags");
        Require(SteamUGC.SetItemContent(update, content), "Could not set release content");
        Require(SteamUGC.SetItemPreview(update, (string)job["preview"]), "Could not set cover image");
        foreach (string path in ((JArray)job["gallery"]).Values<string>()) Require(SteamUGC.AddItemPreviewFile(update, path, EItemPreviewType.k_EItemPreviewType_Image), "Could not add screenshot");
        Require(SteamUGC.SetItemVisibility(update, ERemoteStoragePublishedFileVisibility.k_ERemoteStoragePublishedFileVisibilityPublic), "Could not set public visibility");
        Save("submitting");
        uploadCall = new CallResult<SubmitItemUpdateResult_t>(Uploaded);
        var request = SteamUGC.SubmitItemUpdate(update, File.ReadAllText((string)job["changeNotes"]));
        Require(request != SteamAPICall_t.Invalid, "Steam rejected SubmitItemUpdate");
        uploading = true; uploadCall.Set(request);
    }
    static void Uploaded(SubmitItemUpdateResult_t result, bool io)
    {
        try
        {
            uploading = false;
            if (io) { Finish("outcome-unknown", "Upload response lost. Verify the saved item before retrying."); return; }
            Require(result.m_eResult == EResult.k_EResultOK && result.m_nPublishedFileId == item, "SubmitItemUpdate: " + result.m_eResult);
            state["uploadSucceeded"] = true; Save("uploaded");
            if (result.m_bUserNeedsToAcceptWorkshopLegalAgreement) { Finish("agreement-required", "Upload complete; accept the Steam Workshop legal agreement to make it visible."); return; }
            Query(true);
        }
        catch (Exception e) { Fail(e); }
    }
    static void Downloaded(DownloadItemResult_t result)
    {
            if (result.m_nPublishedFileId != item || result.m_unAppID.m_AppId != 268650) return;
        try
        {
            Require(result.m_eResult == EResult.k_EResultOK, "Verification download failed: " + result.m_eResult);
            Require(SteamUGC.GetItemInstallInfo(item, out var size, out var folder, 4096, out var stamp), "Downloaded content path unavailable");
            Require(Directory.GetDirectories(folder).Length == 0 && Directory.GetFiles(folder).Select(Path.GetFileName).OrderBy(x => x).SequenceEqual(allowed.OrderBy(x => x)), "Downloaded release file list mismatch");
            var hashes = new JObject();
            foreach (string file in allowed)
            {
                string hash = Hash(Path.Combine(content, file)); Require(hash == Hash(Path.Combine(folder, file)), "Downloaded content hash mismatch: " + file); hashes[file] = hash;
            }
            state["verifiedHashes"] = hashes; state["downloadedBytes"] = size; Finish("complete");
        }
        catch (Exception e) { Fail(e); }
    }
}

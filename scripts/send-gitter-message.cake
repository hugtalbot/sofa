#addin "nuget:https://www.nuget.org/api/v2?package=Cake.Gitter&version=2.0.0

Information("This is a 'normal' message...");

var token = EnvironmentVariable("Gitter_token");
var message = EnvironmentVariable("Gitter_message");

var postMessageResult = Gitter.Chat.PostMessage(message:"${message}", messageSettings:new GitterChatMessageSettings {
                                    Token              = "{token}",
                                    RoomId             = "!XWVzIQpwOMjHkyTsNE:gitter.im"
                                    });

if (postMessageResult.Ok)
{
    Information("Message {0} successfully sent", postMessageResult.TimeStamp);
}
else
{
    Error("Failed to send message: {0}", postMessageResult.Error);
}
